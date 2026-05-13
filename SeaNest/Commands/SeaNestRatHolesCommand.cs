using System;
using System.Collections.Generic;
using System.Linq;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using SeaNest.Geometry;

namespace SeaNest.Commands
{
    /// <summary>
    /// SeaNestRatHoles — dual-cut rat-hole + interlocking-slot joinery for
    /// plate × structural-member intersections, using the mid-plane
    /// intersection algorithm (Phase 20a.7).
    ///
    /// For each plate × member pair:
    ///   - Mid-plane of each plate := plane between its two anti-parallel
    ///     large faces (via Phase 14.1.2 twin-pair detection, see
    ///     <see cref="JointGeometryHelpers.GetPlateInfo"/>)
    ///   - Joint centerline := <c>Intersection.PlanePlane(midFrame, midMember)</c>
    ///   - Anchors := where the joint line crosses each plate's mid-plane
    ///     outline (<see cref="JointGeometryHelpers.FindExtremeJointHit"/>)
    ///   - Frame (plate) compound cutter: rectangular slot from bottom
    ///     anchor up to joint midpoint with a half-circle dome cap past
    ///     the chord, plus a full-circle rat-hole cylinder centered on
    ///     the bottom edge
    ///   - Stringer (member) stadium cutter: rounded-bottom slot from
    ///     top anchor down to joint midpoint with the rounded dome past
    ///     the chord (built via <see cref="JointGeometryHelpers.BuildStadiumSlotCutter"/>)
    ///
    /// The full-circle rat hole sidesteps the half-circle's bulge-direction
    /// sign issues — the cylinder is rotationally symmetric, so the half
    /// inside the plate body removes material (= rat hole) and the half
    /// outside does nothing.
    ///
    /// Phase 20a.8: shared joint-geometry helpers extracted into
    /// <see cref="JointGeometryHelpers"/> for reuse by SeaNestSlotsCommand
    /// (Phase 20b, watertight slot variant). This file retains only the
    /// rat-hole-specific compound-cutter logic and the dispatch wiring.
    /// </summary>
    public class SeaNestRatHolesCommand : Command
    {
        public override string EnglishName => "SeaNestRatHoles";

        // Total clearance between plate and slot. Final slot width =
        // matingThickness + ClearanceTotal. Phase 20a.7.3 cuts FINAL
        // dimensions (no kerf compensation) — kerf is downstream CAM's job,
        // and pre-compensating in geometry produced unusably thin slots
        // (e.g., 1/16" wide after Router-kerf subtraction of a 5/16"
        // nominal) plus boolean failures from thin-vs-plate aspect ratios.
        private const double ClearanceTotal = 1.0 / 16.0;   // 1/16" total

        // Reject crossings less than this angle between plate and member faces
        // (i.e. when sin(angleBetween) < 0.05 ≈ 3°). Below this the slot
        // dimensions blow up and the joint is impractical to assemble.
        private const double ParallelRejectSinThreshold = 0.05;

        // Cutter extrusion depth multiplier. Cutter passes through plate's
        // thickness by this factor on each side (so 2× total). Phase 20a.7.4:
        // 4.0 → 2.0 to avoid pathological boolean inputs from huge overshoot.
        private const double CutterOvercutFactor = 2.0;

        // Phase 20a.7.4: offset the cutter centroid off the plate's
        // mid-plane by this multiple of modelTol. Breaks face-coincidence
        // between cutter and plate mid-plane that confused the boolean
        // algorithm into returning the intersection sliver instead of the
        // part-minus-hole.
        private const double CutterMidPlaneOffsetFactor = 2.0;

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            try { return RunInner(doc); }
            catch (Exception ex) { RhinoApp.WriteLine("Crash: " + ex.Message); return Result.Failure; }
        }

        private Result RunInner(RhinoDoc doc)
        {
            bool useInches = doc.ModelUnitSystem == Rhino.UnitSystem.Inches ||
                             doc.ModelUnitSystem == Rhino.UnitSystem.Feet;
            string unitLabel = useInches ? "in" : "mm";
            double inchScale = useInches
                ? 1.0
                : RhinoMath.UnitScale(Rhino.UnitSystem.Inches, doc.ModelUnitSystem);
            double modelTol = doc.ModelAbsoluteTolerance;

            // --- Rat hole radius ---
            double radius = 0.5 * inchScale;
            var gw = new GetNumber();
            gw.SetCommandPrompt(string.Format("Rat hole radius ({0})", unitLabel));
            gw.SetDefaultNumber(radius);
            gw.SetLowerLimit(modelTol, false);
            gw.AcceptNothing(true);
            if (gw.Get() == GetResult.Number) radius = gw.Number();

            double clearance = ClearanceTotal * inchScale;

            RhinoApp.WriteLine(string.Format(
                "SeaNest Rat Holes: radius {0:G3} {1}, clearance {2:G3} {1}.",
                radius, unitLabel, clearance));

            // --- Select plates ---
            var goPlates = new GetObject();
            goPlates.SetCommandPrompt("Select plates");
            goPlates.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
            goPlates.GroupSelect = true;
            goPlates.SubObjectSelect = false;
            goPlates.EnablePreSelect(false, false);
            goPlates.GetMultiple(1, 0);
            if (goPlates.CommandResult() != Result.Success) return goPlates.CommandResult();

            var plates = new List<Brep>();
            var plateIds = new List<Guid>();
            var plateIdSet = new HashSet<Guid>();
            for (int i = 0; i < goPlates.ObjectCount; i++)
            {
                var obj = goPlates.Object(i);
                var b = obj.Brep();
                if (b == null)
                {
                    var ext = obj.Geometry() as Extrusion;
                    if (ext != null) b = ext.ToBrep();
                }
                if (b != null && b.IsValid)
                {
                    plates.Add(b.DuplicateBrep());
                    plateIds.Add(obj.ObjectId);
                    plateIdSet.Add(obj.ObjectId);
                }
            }

            // --- Select structural members ---
            var goMembers = new GetObject();
            goMembers.SetCommandPrompt("Select structural members");
            goMembers.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
            goMembers.GroupSelect = true;
            goMembers.SubObjectSelect = false;
            goMembers.EnablePreSelect(false, false);
            goMembers.GetMultiple(1, 0);
            if (goMembers.CommandResult() != Result.Success) return goMembers.CommandResult();

            var members = new List<Brep>();
            var memberIds = new List<Guid>();
            int selfPickedCount = 0;
            for (int i = 0; i < goMembers.ObjectCount; i++)
            {
                var obj = goMembers.Object(i);
                if (plateIdSet.Contains(obj.ObjectId))
                {
                    selfPickedCount++;
                    continue;
                }
                var b = obj.Brep();
                if (b == null)
                {
                    var ext = obj.Geometry() as Extrusion;
                    if (ext != null) b = ext.ToBrep();
                }
                if (b != null && b.IsValid)
                {
                    members.Add(b.DuplicateBrep());
                    memberIds.Add(obj.ObjectId);
                }
            }
            if (selfPickedCount > 0)
            {
                RhinoApp.WriteLine(string.Format(
                    "{0} plate(s) re-picked as members — filtered out (plates can't cut themselves).",
                    selfPickedCount));
            }

            if (plates.Count == 0 || members.Count == 0)
            {
                RhinoApp.WriteLine("Need at least 1 plate and 1 structural member.");
                return Result.Failure;
            }

            // --- Per-pair joint compute: accumulate cutters per part ---
            var plateCutters = new List<List<Brep>>();
            var memberCutters = new List<List<Brep>>();
            for (int i = 0; i < plates.Count; i++) plateCutters.Add(new List<Brep>());
            for (int i = 0; i < members.Count; i++) memberCutters.Add(new List<Brep>());

            int pairsSkipped = 0;

            for (int p = 0; p < plates.Count; p++)
            {
                var plate = plates[p];
                var plateBBox = plate.GetBoundingBox(true);

                for (int m = 0; m < members.Count; m++)
                {
                    var member = members[m];
                    var memberBBox = member.GetBoundingBox(true);
                    var overlap = BoundingBox.Intersection(plateBBox, memberBBox);
                    if (!overlap.IsValid) continue;

                    JointCutResult joint;
                    try
                    {
                        joint = BuildFrameStringerJointCutters(
                            plate, member,
                            clearance, radius,
                            modelTol);
                    }
                    catch (Exception ex)
                    {
                        RhinoApp.WriteLine(string.Format(
                            "  Plate {0} × Member {1}: {2} — skipped.",
                            p + 1, m + 1, ex.Message));
                        pairsSkipped++;
                        continue;
                    }

                    if (joint?.FrameCutters != null)
                        plateCutters[p].AddRange(joint.FrameCutters);
                    if (joint?.StringerCutters != null)
                        memberCutters[m].AddRange(joint.StringerCutters);
                }
            }

            // --- Apply accumulated cuts (silent-skip on empty per Phase 20a.2) ---
            int platesCut = 0;
            int totalPlateCuts = 0;
            int membersCut = 0;
            int totalMemberCuts = 0;
            int booleanFailures = 0;

            for (int p = 0; p < plates.Count; p++)
            {
                if (plateCutters[p].Count == 0) continue;
                var result = JointGeometryHelpers.ApplyCutters(plates[p], plateCutters[p], modelTol);
                if (result == null)
                {
                    RhinoApp.WriteLine(string.Format(
                        "  Plate {0}: boolean difference failed ({1} cutter(s)) — left unchanged.",
                        p + 1, plateCutters[p].Count));
                    booleanFailures++;
                    continue;
                }
                if (doc.Objects.Replace(plateIds[p], result))
                {
                    platesCut++;
                    totalPlateCuts += plateCutters[p].Count;
                }
                else
                {
                    RhinoApp.WriteLine(string.Format(
                        "  Plate {0}: doc.Objects.Replace failed.", p + 1));
                    booleanFailures++;
                }
            }

            for (int m = 0; m < members.Count; m++)
            {
                if (memberCutters[m].Count == 0) continue;
                var result = JointGeometryHelpers.ApplyCutters(members[m], memberCutters[m], modelTol);
                if (result == null)
                {
                    RhinoApp.WriteLine(string.Format(
                        "  Member {0}: boolean difference failed ({1} cutter(s)) — left unchanged.",
                        m + 1, memberCutters[m].Count));
                    booleanFailures++;
                    continue;
                }
                if (doc.Objects.Replace(memberIds[m], result))
                {
                    membersCut++;
                    totalMemberCuts += memberCutters[m].Count;
                }
                else
                {
                    RhinoApp.WriteLine(string.Format(
                        "  Member {0}: doc.Objects.Replace failed.", m + 1));
                    booleanFailures++;
                }
            }

            RhinoApp.WriteLine("==========================================");
            RhinoApp.WriteLine(string.Format(
                "  SeaNest Rat Holes: {0} plate(s) cut with {1} cutter(s); " +
                "{2} member(s) cut with {3} cutter(s).",
                platesCut, totalPlateCuts, membersCut, totalMemberCuts));
            if (pairsSkipped > 0)
                RhinoApp.WriteLine(string.Format(
                    "  Skipped {0} pair(s) due to joint-geometry issues (see messages above).",
                    pairsSkipped));
            if (booleanFailures > 0)
                RhinoApp.WriteLine(string.Format(
                    "  {0} boolean failure(s) — affected parts left unchanged.", booleanFailures));
            RhinoApp.WriteLine("==========================================");

            doc.Views.Redraw();
            return Result.Success;
        }

        // ---------------------------------------------------------------
        // Joint-cutter dispatch (rat-hole-specific)
        // ---------------------------------------------------------------
        //
        // Phase 20a.8: shared joint geometry primitives (GetPlateInfo,
        // SectionPlateAtMidPlane, FindExtremeJointHit, etc.) live in
        // JointGeometryHelpers. This file retains the rat-hole-specific
        // compound-cutter builder + the dispatch that wires it together
        // with the shared stringer stadium cutter.

        /// <summary>
        /// Main per-pair joint-compute. Returns a <see cref="JointCutResult"/>
        /// with both the frame compound cutters (rectangle slot + full-circle
        /// rat-hole) and the stringer stadium cutter. Shared joint-geometry
        /// math (mid-plane detection, joint centerline, anchor finding) is
        /// delegated to <see cref="JointGeometryHelpers"/>.
        /// </summary>
        private static JointCutResult BuildFrameStringerJointCutters(
            Brep frame, Brep stringer,
            double clearance, double frameRatHoleRadius,
            double tol)
        {
            Vector3d worldUp = Vector3d.ZAxis;

            PlateInfo frameInfo = JointGeometryHelpers.GetPlateInfo(frame, tol);
            PlateInfo stringerInfo = JointGeometryHelpers.GetPlateInfo(stringer, tol);

            Vector3d nf = frameInfo.Normal; nf.Unitize();
            Vector3d ns = stringerInfo.Normal; ns.Unitize();

            // Joint centerline: intersection of the two mid-planes.
            double sinAngle = Vector3d.CrossProduct(nf, ns).Length;
            if (sinAngle < ParallelRejectSinThreshold)
                throw new Exception("frame and member are nearly parallel");

            if (!Rhino.Geometry.Intersect.Intersection.PlanePlane(
                    frameInfo.MidPlane, stringerInfo.MidPlane, out Line jointLine))
                throw new Exception("mid-planes do not intersect");

            Vector3d upDir = jointLine.Direction;
            if (!upDir.Unitize())
                throw new Exception("joint centerline has zero length");
            if (upDir * worldUp < 0.0) upDir.Reverse();

            // Section both plates at their own mid-planes.
            Curve[] frameOutline = JointGeometryHelpers.SectionPlateAtMidPlane(frame, frameInfo.MidPlane, tol);
            Curve[] stringerOutline = JointGeometryHelpers.SectionPlateAtMidPlane(stringer, stringerInfo.MidPlane, tol);
            if (frameOutline.Length == 0)
                throw new Exception("frame mid-plane section returned no curves");
            if (stringerOutline.Length == 0)
                throw new Exception("stringer mid-plane section returned no curves");

            // Anchors: where the joint line crosses each outline.
            Point3d frameBottomAnchor =
                JointGeometryHelpers.FindExtremeJointHit(frameOutline, jointLine, upDir, findTop: false, tol);
            Point3d stringerTopAnchor =
                JointGeometryHelpers.FindExtremeJointHit(stringerOutline, jointLine, upDir, findTop: true, tol);
            Point3d stringerBottomAnchor =
                JointGeometryHelpers.FindExtremeJointHit(stringerOutline, jointLine, upDir, findTop: false, tol);

            Point3d stringerMidPoint = new Point3d(
                (stringerTopAnchor.X + stringerBottomAnchor.X) * 0.5,
                (stringerTopAnchor.Y + stringerBottomAnchor.Y) * 0.5,
                (stringerTopAnchor.Z + stringerBottomAnchor.Z) * 0.5);

            // Cut heights along the joint centerline. These are CHORD
            // positions (where each stadium's straight portion ends and
            // the dome cap begins) — NOT the dome apex.
            double frameCutHeight = JointGeometryHelpers.DistanceAlong(frameBottomAnchor, stringerMidPoint, upDir);
            double stringerSlotDepth = JointGeometryHelpers.DistanceAlong(stringerMidPoint, stringerTopAnchor, upDir);

            if (frameCutHeight <= tol)
                throw new Exception("frame cut height ≤ 0");
            if (stringerSlotDepth <= tol)
                throw new Exception("stringer slot depth ≤ 0");

            // Slot widths: final = mating thickness + clearance, scaled by
            // 1/sinAngle for oblique crossings.
            double frameSlotWidth = (stringerInfo.Thickness + clearance) / sinAngle;
            double stringerSlotWidth = (frameInfo.Thickness + clearance) / sinAngle;

            if (frameSlotWidth <= tol) throw new Exception("frame slot width ≤ 0");
            if (stringerSlotWidth <= tol) throw new Exception("stringer slot width ≤ 0");

            // In-plane width axes (perpendicular to upDir, in each plate's plane).
            Vector3d frameWidthDir = Vector3d.CrossProduct(upDir, nf);
            if (!frameWidthDir.Unitize()) throw new Exception("frame width axis degenerate");
            Vector3d stringerWidthDir = Vector3d.CrossProduct(upDir, ns);
            if (!stringerWidthDir.Unitize()) throw new Exception("stringer width axis degenerate");

            double cutterDepth = Math.Max(frameInfo.Thickness, stringerInfo.Thickness)
                                 * CutterOvercutFactor;

            // Offset each cutter's center off the plate's mid-plane to
            // break face-coincidence with Rhino's boolean algorithm.
            double midPlaneOffset = tol * CutterMidPlaneOffsetFactor;
            Point3d frameAnchorForCut = frameBottomAnchor + nf * midPlaneOffset;
            Point3d stringerAnchorForCut = stringerTopAnchor + ns * midPlaneOffset;

            Brep[] frameCutters = BuildFrameCompoundCutters(
                frameAnchorForCut, nf, frameWidthDir, upDir,
                frameSlotWidth, frameCutHeight, frameRatHoleRadius,
                cutterDepth, tol);

            // Member-side stadium is the generic slot cutter — shared with
            // SeaNestSlotsCommand via JointGeometryHelpers.
            Brep[] stringerCutters = JointGeometryHelpers.BuildStadiumSlotCutter(
                stringerAnchorForCut, ns, stringerWidthDir, upDir,
                stringerSlotWidth, stringerSlotDepth,
                cutterDepth, tol);

            return new JointCutResult
            {
                FrameCutters = frameCutters,
                StringerCutters = stringerCutters,
                FrameBottomAnchor = frameBottomAnchor,
                StringerTopAnchor = stringerTopAnchor,
                StringerMidPoint = stringerMidPoint,
                UpDir = upDir,
                FrameNormal = nf,
                StringerNormal = ns,
            };
        }

        /// <summary>
        /// Build the frame's compound cutter set: a stadium-shaped slot
        /// extending upward from the bottom anchor (rectangle + half-circle
        /// dome cap past the chord into the stringer's body), plus a
        /// full-circle rat-hole cylinder centered ON the frame's bottom
        /// edge. The half of the cylinder inside the plate body becomes
        /// the rat-hole bite; the half outside is geometrically harmless.
        ///
        /// Rat-hole-specific (stays in SeaNestRatHolesCommand). The
        /// stringer's stadium cutter is built via the shared
        /// <see cref="JointGeometryHelpers.BuildStadiumSlotCutter"/>.
        /// </summary>
        private static Brep[] BuildFrameCompoundCutters(
            Point3d anchor,
            Vector3d frameNormal,
            Vector3d frameWidthDir,
            Vector3d upDir,
            double slotWidth,
            double slotHeight,
            double ratHoleRadius,
            double cutterDepth,
            double tol)
        {
            var cutters = new List<Brep>();

            // Slot plane: X=frameWidthDir, Y=upDir, origin=anchor (on frame
            // bottom edge). Slot extends UP from the bottom edge to the joint
            // midpoint; bottom of profile sits BELOW the bottom edge by
            // cutterDepth so the cutter overcuts past the bottom and joins
            // the rat-hole cylinder's overcut region without residual edge
            // material.
            //
            // Stadium: chord at Y = slotHeight (joint center), dome extending
            // PAST the chord into the stringer's body (apex at Y = H + r).
            // The dome is a cavity inside the stringer's solid material —
            // it can't collide with the stringer's downward dome (which is
            // itself a cavity inside the frame's material).
            var slotPlane = JointGeometryHelpers.MakeSafeCutPlane(anchor, frameWidthDir, upDir);
            double r = slotWidth / 2.0;
            double H = slotHeight;

            //   p0 = (-r, -cutterDepth)  bottom-left, below frame's bottom edge
            //   p1 = (-r, H)             left chord endpoint at joint center
            //   arcMid = (0, H + r)      apex of half-circle BEYOND the chord
            //   p2 = (+r, H)             right chord endpoint at joint center
            //   p3 = (+r, -cutterDepth)  bottom-right
            //   close p3 → p0            bottom edge (below frame)
            var p0 = slotPlane.PointAt(-r, -cutterDepth);
            var p1 = slotPlane.PointAt(-r, H);
            var arcMid = slotPlane.PointAt(0, H + r);
            var p2 = slotPlane.PointAt(+r, H);
            var p3 = slotPlane.PointAt(+r, -cutterDepth);

            var leftSide = new LineCurve(p0, p1);
            var topArc = new ArcCurve(new Arc(p1, arcMid, p2));
            if (topArc.IsValid)
            {
                var rightSide = new LineCurve(p2, p3);
                var bottom = new LineCurve(p3, p0);

                var poly = new PolyCurve();
                poly.Append(leftSide);
                poly.Append(topArc);
                poly.Append(rightSide);
                poly.Append(bottom);
                if (poly.MakeClosed(0.001))
                {
                    var slotCutter = JointGeometryHelpers.ExtrudeClosedPlanarCurve(poly, frameNormal, cutterDepth);
                    if (slotCutter != null) cutters.Add(slotCutter);
                }
            }

            // Full-circle rat hole at the bottom anchor — rotationally
            // symmetric, sidesteps half-circle orientation issues.
            if (ratHoleRadius > tol)
            {
                var ratHoleCircleFrame = new Plane(anchor, frameNormal);
                var circle = new Circle(ratHoleCircleFrame, ratHoleRadius);
                var cyl = new Cylinder(circle, cutterDepth);
                var cylBrep = cyl.ToBrep(true, true);
                if (cylBrep != null && cylBrep.IsValid)
                {
                    cylBrep.Translate(frameNormal * (-cutterDepth / 2.0));
                    JointGeometryHelpers.EnsureOutwardOrientation(cylBrep);
                    cutters.Add(cylBrep);
                }
            }

            return cutters.ToArray();
        }
    }
}
