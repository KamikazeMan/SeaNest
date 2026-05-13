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
            // Phase 20c.4 — reverted to world-Z sign-correction. Phase
            // 20c.3's body-relative attempt (towardMember = memberCenter -
            // jointMid) produced uniformly flipped cuts because:
            //   (a) jointLine.PointAt(0.5) is at an arbitrary parametric
            //       position on the line (Intersection.PlanePlane's From/To
            //       are not geometrically anchored), so jointMid can land
            //       far from the actual joint contact;
            //   (b) Even with a corrected jointMid via ClosestParameter,
            //       the resulting towardMember is perpendicular to the
            //       joint-line direction (closest-point projection
            //       minimizes distance → vector is perpendicular to line),
            //       so upDir * towardMember = 0 and sign-correction is
            //       undefined / driven by float noise.
            //
            // World-Z works for the typical vertical-member geometry that
            // SeaNest targets. The "curved members mirrored" symptom that
            // motivated 20c.3 was actually Phase 20c.1's unfiltered BrepBrep
            // fallback returning spurious anchors — fixed by 20c.2's
            // proximity filter, which remains in place. Horizontal-member
            // geometry (joint line perpendicular to Z) is a separate
            // unresolved case for any future phase.
            if (upDir * worldUp < 0.0) upDir.Reverse();

            // Section both plates at their own mid-planes.
            Curve[] frameOutline = JointGeometryHelpers.SectionPlateAtMidPlane(frame, frameInfo.MidPlane, tol);
            Curve[] stringerOutline = JointGeometryHelpers.SectionPlateAtMidPlane(stringer, stringerInfo.MidPlane, tol);
            if (frameOutline.Length == 0)
                throw new Exception("frame mid-plane section returned no curves");
            if (stringerOutline.Length == 0)
                throw new Exception("stringer mid-plane section returned no curves");

            // Anchors: where the joint line crosses each outline.
            // Phase 20c.1: each anchor lookup falls back to BrepBrep
            // intersection if the mid-plane outline misses.
            // Phase 20c.2: BrepBrep fallback is filtered by proximity to
            // the joint line — discards spurious brush-contact points
            // for curved members.
            // Phase 20c.3: cutoff = 10× MAX thickness (was 5× sum). The
            // max-based formula tracks the larger part's geometry more
            // faithfully when thicknesses differ — a thick plate's
            // legitimate curvature-displaced joint can spread further
            // than the sum-based formula would tolerate.
            double maxJointDistance = Math.Max(frameInfo.Thickness, stringerInfo.Thickness) * 10.0;

            Point3d frameBottomAnchor =
                ResolveAnchorWithFallback(frameOutline, jointLine, upDir, findTop: false,
                    frame, stringer, "frame", "bottom", maxJointDistance, tol);
            Point3d stringerTopAnchor =
                ResolveAnchorWithFallback(stringerOutline, jointLine, upDir, findTop: true,
                    stringer, frame, "stringer", "top", maxJointDistance, tol);
            Point3d stringerBottomAnchor =
                ResolveAnchorWithFallback(stringerOutline, jointLine, upDir, findTop: false,
                    stringer, frame, "stringer", "bottom", maxJointDistance, tol);

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

            // Phase 20b.1b: frame's stadium and member's stadium both go
            // through the shared generic stadium-cutter primitive. The
            // direction the slot extends from each anchor INTO the body
            // is the only thing that differs:
            //   - Frame: anchor on bottom edge → slot extends +upDir
            //   - Member: anchor on top edge → slot extends -upDir
            // Rat-hole cylinder is rat-hole-specific (BuildRatHoleCylinder
            // below); it's added to the frame's cutter list alongside the
            // stadium.
            var frameStadium = JointGeometryHelpers.BuildStadiumSlotCutter(
                frameAnchorForCut, nf, frameWidthDir, upDir,
                frameSlotWidth, frameCutHeight,
                cutterDepth, tol);
            var ratHole = BuildRatHoleCylinder(
                frameAnchorForCut, nf, frameRatHoleRadius, cutterDepth, tol);
            var frameCutters = ratHole != null
                ? frameStadium.Concat(new[] { ratHole }).ToArray()
                : frameStadium;

            Brep[] stringerCutters = JointGeometryHelpers.BuildStadiumSlotCutter(
                stringerAnchorForCut, ns, stringerWidthDir, -upDir,
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
        /// Build a full-circle rat-hole cylinder cutter centered on the
        /// frame's bottom edge anchor. Rat-hole-specific (no
        /// SeaNestSlotsCommand counterpart — slots don't include a
        /// rat-hole).
        ///
        /// The cylinder is rotationally symmetric, so the half inside the
        /// plate body becomes the rat-hole bite and the half outside is
        /// geometrically harmless. Cylinder axis = frame normal; depth =
        /// cutterDepth (passes through the plate's full thickness with
        /// overcut on both sides).
        ///
        /// Returns null on zero/sub-tolerance radius (caller treats as
        /// "no rat-hole") or on Brep construction failure.
        /// </summary>
        private static Brep BuildRatHoleCylinder(
            Point3d anchor, Vector3d frameNormal,
            double ratHoleRadius, double cutterDepth, double tol)
        {
            if (ratHoleRadius <= tol) return null;
            var circlePlane = new Plane(anchor, frameNormal);
            var circle = new Circle(circlePlane, ratHoleRadius);
            var cyl = new Cylinder(circle, cutterDepth);
            var cylBrep = cyl.ToBrep(true, true);
            if (cylBrep == null || !cylBrep.IsValid) return null;
            cylBrep.Translate(frameNormal * (-cutterDepth / 2.0));
            JointGeometryHelpers.EnsureOutwardOrientation(cylBrep);
            return cylBrep;
        }

        /// <summary>
        /// Phase 20c.1 — resolve a joint anchor via the mid-plane outline
        /// primary; on miss, log + fall back to BrepBrep intersection
        /// between the two parts. Throws when both fail (caught by the
        /// per-pair loop which logs the skip with plate/member indices).
        ///
        /// <paramref name="anchorPart"/> is the part whose anchor we
        /// want; <paramref name="otherPart"/> is the part it touches.
        /// The fallback uses BrepBrep on those two parts (symmetric, but
        /// the labels in the diagnostic refer to <paramref name="partLabel"/>
        /// and <paramref name="positionLabel"/>).
        /// </summary>
        private static Point3d ResolveAnchorWithFallback(
            Curve[] outline, Line jointLine, Vector3d upDir, bool findTop,
            Brep anchorPart, Brep otherPart,
            string partLabel, string positionLabel,
            double maxJointDistance, double tol)
        {
            var primary = JointGeometryHelpers.FindExtremeJointHit(
                outline, jointLine, upDir, findTop, tol);
            if (primary.HasValue) return primary.Value;

            RhinoApp.WriteLine(string.Format(
                "    joint line missed {0} outline ({1}) — attempting BrepBrep fallback near joint line (within {2:G3}).",
                partLabel, positionLabel, maxJointDistance));

            var fallback = JointGeometryHelpers.FindExtremeIntersectionNearJointLine(
                anchorPart, otherPart, upDir, findHigh: findTop,
                jointLine, maxJointDistance, tol);
            if (fallback.HasValue)
            {
                var pt = fallback.Value;
                RhinoApp.WriteLine(string.Format(
                    "    fallback anchor ({0} {1}): ({2:F2}, {3:F2}, {4:F2}).",
                    partLabel, positionLabel, pt.X, pt.Y, pt.Z));
                return pt;
            }

            throw new Exception(string.Format(
                "BrepBrep fallback found no intersection within {0:G3} of joint line ({1} {2} anchor)",
                maxJointDistance, partLabel, positionLabel));
        }
    }
}
