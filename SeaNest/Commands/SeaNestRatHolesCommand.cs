using System;
using System.Collections.Generic;
using System.Linq;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Input;
using Rhino.Input.Custom;

namespace SeaNest.Commands
{
    /// <summary>
    /// Phase 20a.7 — SeaNestRatHoles dual-cut using the mid-plane intersection
    /// algorithm. Replaces the prior anchor-finding heuristics (FindEdgeAnchor,
    /// ComputeAlongMember, body-reference sign corrections) with a single
    /// well-defined geometric primitive: the joint centerline is the
    /// intersection of the two plate mid-planes; the bottom/top anchors are
    /// where that line hits each plate's mid-plane outline.
    ///
    /// For each plate × structural-member pair:
    ///   - Mid-plane of plate := plane between its two anti-parallel large faces
    ///   - Mid-plane of member := same
    ///   - Joint centerline := Intersection.PlanePlane(midPlate, midMember)
    ///   - Plate bottom anchor := bottommost intersection of joint line with
    ///     the plate's mid-plane outline (Intersection.BrepPlane)
    ///   - Member top anchor := topmost equivalent on the member
    ///   - Plate gets a compound cutter: rectangular slot (anchor → midpoint)
    ///     plus a full-circle rat-hole cylinder at the anchor
    ///   - Member gets a stadium cutter (anchor → midpoint, with a rounded
    ///     bottom for stress relief)
    ///
    /// The full-circle rat hole sidesteps the half-circle's bulge-direction
    /// sign issues — the cylinder is rotationally symmetric, so the half
    /// inside the plate body removes material (= rat hole) and the half
    /// outside does nothing.
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

        // Phase 20a.7.8: AssemblyClearance const removed. The dome caps now
        // extend PAST the chord into solid material (each side's cut goes
        // a half-circle beyond the joint centerline). Both chord lines
        // land exactly at the joint center — even and aligned — and the
        // domes themselves are cavities inside their parent solids, so
        // they never physically collide regardless of fit tolerance.

        // Reject crossings less than this angle between plate and member faces
        // (i.e. when sin(angleBetween) < 0.05 ≈ 3°). Below this the slot
        // dimensions blow up and the joint is impractical to assemble.
        private const double ParallelRejectSinThreshold = 0.05;

        // Cutter extrusion depth multiplier. Cutter passes through plate's
        // thickness by this factor on each side (so 2× total). Phase 20a.7.4:
        // reduced from 4.0 to 2.0 — the original 4× margin had cutters
        // overshooting the plate faces by 1.5× thickness each, which
        // (combined with mid-plane face-coincidence) tripped Rhino's
        // CreateBooleanDifference into returning the intersection slice
        // instead of the part-with-hole. 2× still cleanly punches through
        // (0.125" overhang per face on a 0.25" plate) without producing
        // pathological boolean inputs.
        private const double CutterOvercutFactor = 2.0;

        // Phase 20a.7.4: offset the cutter centroid off the plate's
        // mid-plane by this multiple of modelTol. Without it, the cutter is
        // exactly centered on the plate's mid-plane — and parallel/coincident
        // internal planes confuse the boolean algorithm, causing it to keep
        // the intersection sliver instead of the part-minus-hole. The offset
        // is along the plate's normal, so the cutter shifts perpendicular to
        // the cut plane while still spanning the plate's full thickness.
        // 2× modelTol is small enough to be visually unmeasurable on plate
        // work (0.02" at typical doc tol) but large enough to break the
        // coincident-face condition.
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

            // Phase 20a.7.3: no cutting-method prompt. We cut FINAL geometry
            // dimensions and let downstream CAM software handle kerf at
            // toolpath generation. Drop the Router/Waterjet/Plasma selection
            // along with the kerf constants — consistent with Phase 17 export
            // and Phase 19b scribe lines, which also don't model kerf.
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
            int pairsProcessed = 0;

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
                    pairsProcessed++;
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
                var result = ApplyCutters(plates[p], plateCutters[p], modelTol);
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
                var result = ApplyCutters(members[m], memberCutters[m], modelTol);
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
        // Joint geometry primitives
        // ---------------------------------------------------------------

        /// <summary>
        /// Plate-as-thin-sheet metadata: the mid-plane between the two large
        /// anti-parallel faces, the mid-plane normal (= the plate's "thin"
        /// direction), and the perpendicular thickness.
        /// </summary>
        private class PlateInfo
        {
            public Plane MidPlane;
            public Vector3d Normal;
            public double Thickness;
        }

        /// <summary>
        /// Output of <see cref="BuildFrameStringerJointCutters"/> — one set of
        /// cutters for the frame (plate) and one for the stringer (member),
        /// plus the joint geometry for diagnostics.
        /// </summary>
        private class JointCutResult
        {
            public Brep[] FrameCutters;
            public Brep[] StringerCutters;
            public Point3d FrameBottomAnchor;
            public Point3d StringerTopAnchor;
            public Point3d StringerMidPoint;
            public Vector3d UpDir;
            public Vector3d FrameNormal;
            public Vector3d StringerNormal;
        }

        /// <summary>
        /// Phase 20a.7 — main per-pair joint-compute. Computes the joint
        /// centerline as Intersection.PlanePlane(frameMid, stringerMid),
        /// then finds the frame's bottom anchor and the stringer's top/bottom
        /// anchors by intersecting the joint line with each plate's mid-plane
        /// outline (Intersection.BrepPlane). Builds:
        ///   - Frame compound cutter: rectangle slot from anchor → stringer
        ///     midpoint, plus full-circle rat-hole cylinder at the anchor
        ///   - Stringer stadium cutter: rounded slot from stringer top anchor
        ///     down to the midpoint
        /// </summary>
        private static JointCutResult BuildFrameStringerJointCutters(
            Brep frame, Brep stringer,
            double clearance, double frameRatHoleRadius,
            double tol)
        {
            Vector3d worldUp = Vector3d.ZAxis;

            PlateInfo frameInfo = GetPlateInfoFromLargestPlanarFaces(frame, tol);
            PlateInfo stringerInfo = GetPlateInfoFromLargestPlanarFaces(stringer, tol);

            Vector3d nf = frameInfo.Normal; nf.Unitize();
            Vector3d ns = stringerInfo.Normal; ns.Unitize();

            // Joint centerline: intersection of the two mid-planes.
            double sinAngle = Vector3d.CrossProduct(nf, ns).Length;
            if (sinAngle < ParallelRejectSinThreshold)
                throw new Exception("frame and member are nearly parallel");

            if (!Intersection.PlanePlane(frameInfo.MidPlane, stringerInfo.MidPlane, out Line jointLine))
                throw new Exception("mid-planes do not intersect");

            Vector3d upDir = jointLine.Direction;
            if (!upDir.Unitize())
                throw new Exception("joint centerline has zero length");
            // Orient up toward world +Z. For typical boat-orientation
            // workflows this is the right convention. For unusually-rotated
            // geometry the user can pre-orient the document.
            if (upDir * worldUp < 0.0) upDir.Reverse();

            // Section both plates at their own mid-planes.
            Curve[] frameOutline = SectionPlateAtMidPlane(frame, frameInfo.MidPlane, tol);
            Curve[] stringerOutline = SectionPlateAtMidPlane(stringer, stringerInfo.MidPlane, tol);
            if (frameOutline.Length == 0)
                throw new Exception("frame mid-plane section returned no curves");
            if (stringerOutline.Length == 0)
                throw new Exception("stringer mid-plane section returned no curves");

            // Anchors: where the joint line crosses each outline.
            Point3d frameBottomAnchor =
                FindExtremeJointHit(frameOutline, jointLine, upDir, findTop: false, tol);
            Point3d stringerTopAnchor =
                FindExtremeJointHit(stringerOutline, jointLine, upDir, findTop: true, tol);
            Point3d stringerBottomAnchor =
                FindExtremeJointHit(stringerOutline, jointLine, upDir, findTop: false, tol);

            Point3d stringerMidPoint = new Point3d(
                (stringerTopAnchor.X + stringerBottomAnchor.X) * 0.5,
                (stringerTopAnchor.Y + stringerBottomAnchor.Y) * 0.5,
                (stringerTopAnchor.Z + stringerBottomAnchor.Z) * 0.5);

            // Cut heights along the joint centerline. Phase 20a.7.8 — these
            // are CHORD positions (where each stadium's straight portion
            // ends and the dome cap begins), NOT the dome apex. Each
            // dome extends past its chord into solid material on the far
            // side of the joint center. Both chords land at the joint
            // center, evenly aligned.
            double frameCutHeight = DistanceAlong(frameBottomAnchor, stringerMidPoint, upDir);
            double stringerSlotDepth = DistanceAlong(stringerMidPoint, stringerTopAnchor, upDir);

            if (frameCutHeight <= tol)
                throw new Exception("frame cut height ≤ 0");
            if (stringerSlotDepth <= tol)
                throw new Exception("stringer slot depth ≤ 0");

            // Slot widths: final = mating thickness + clearance, scaled by
            // 1/sinAngle for oblique crossings (the perpendicular plate thickness
            // projects wider along the slot's width axis). Phase 20a.7.3: no
            // kerf subtraction — CAM handles it at toolpath time.
            double frameSlotWidth = (stringerInfo.Thickness + clearance) / sinAngle;
            double stringerSlotWidth = (frameInfo.Thickness + clearance) / sinAngle;

            if (frameSlotWidth <= tol)
                throw new Exception("frame slot width ≤ 0");
            if (stringerSlotWidth <= tol)
                throw new Exception("stringer slot width ≤ 0");

            // In-plane width axes (perpendicular to upDir, in each plate's plane).
            Vector3d frameWidthDir = Vector3d.CrossProduct(upDir, nf);
            if (!frameWidthDir.Unitize()) throw new Exception("frame width axis degenerate");
            Vector3d stringerWidthDir = Vector3d.CrossProduct(upDir, ns);
            if (!stringerWidthDir.Unitize()) throw new Exception("stringer width axis degenerate");

            double cutterDepth = Math.Max(frameInfo.Thickness, stringerInfo.Thickness)
                                 * CutterOvercutFactor;

            // Phase 20a.7.4: offset each cutter's center off the plate's
            // mid-plane (perpendicular to the plate face, along the plate's
            // normal). Breaks face-coincidence between cutter and plate
            // mid-plane — see CutterMidPlaneOffsetFactor doc comment.
            double midPlaneOffset = tol * CutterMidPlaneOffsetFactor;
            Point3d frameAnchorForCut = frameBottomAnchor + nf * midPlaneOffset;
            Point3d stringerAnchorForCut = stringerTopAnchor + ns * midPlaneOffset;

            Brep[] frameCutters = BuildFrameCompoundCutters(
                frameAnchorForCut, nf, frameWidthDir, upDir,
                frameSlotWidth, frameCutHeight, frameRatHoleRadius,
                cutterDepth, tol);

            Brep[] stringerCutters = BuildStringerStadiumCutter(
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
        /// Identify a thin plate's mid-plane.
        ///
        /// Phase 20a.7.1 — primary path uses
        /// <see cref="SeaNest.RhinoAdapters.BrepFlattener.FindTwinPlateFace"/>,
        /// Phase 14.1.2's twin-pair detection that's already tuned for marine
        /// plate geometry (10° anti-parallel tolerance, 0.7 area-ratio
        /// minimum, 0.1 separation-ratio maximum). The returned `Plane` IS
        /// the mid-plane — origin is the midpoint of the two face centroids,
        /// normal averages the two opposing face normals. Thickness is
        /// derived as 2× the perpendicular distance from the returned face's
        /// centroid to the mid-plane.
        ///
        /// Fallback path (when twin-pair fails): use the largest planar face
        /// and synthesize a mid-plane by computing the brep's bbox extents
        /// along that face's normal. Mid-plane origin sits at the midpoint
        /// of those extents. Thickness = extent range. Less robust for
        /// non-axis-aligned or odd-shaped plates but provides a working
        /// answer when the twin-pair criteria reject (e.g., heavily curved
        /// plates where the two large faces aren't planar).
        /// </summary>
        private static PlateInfo GetPlateInfoFromLargestPlanarFaces(Brep brep, double tol)
        {
            // Primary: Phase 14.1.2 twin-pair detection.
            var twin = SeaNest.RhinoAdapters.BrepFlattener.FindTwinPlateFace(brep, tol);
            if (twin != null)
            {
                var face = twin.Value.Face;
                var midPlane = twin.Value.Plane;
                Vector3d normal = midPlane.Normal;
                if (!normal.Unitize()) throw new Exception("twin-pair mid-plane normal degenerate");

                // Thickness = 2 × perpendicular distance from face centroid
                // to the mid-plane (the twin face sits at the same distance
                // on the other side by construction).
                double thickness;
                using (var faceBrep = face.DuplicateFace(false))
                {
                    var amp = AreaMassProperties.Compute(faceBrep);
                    if (amp == null) throw new Exception("twin-pair face centroid failed");
                    double signedDist = (amp.Centroid - midPlane.Origin) * normal;
                    thickness = 2.0 * Math.Abs(signedDist);
                }
                if (thickness < tol)
                    throw new Exception("twin-pair thickness ≤ tolerance");

                return new PlateInfo
                {
                    MidPlane = midPlane,
                    Normal = normal,
                    Thickness = thickness,
                };
            }

            // Fallback: largest planar face + bbox extents along its normal.
            BrepFace largest = null;
            Plane largestPlane = Plane.Unset;
            double maxArea = 0;
            foreach (var face in brep.Faces)
            {
                if (!face.IsPlanar(tol * 10.0)) continue;
                if (!face.TryGetPlane(out Plane p, tol * 10.0)) continue;
                try
                {
                    using (var fb = face.DuplicateFace(false))
                    {
                        var amp = AreaMassProperties.Compute(fb);
                        if (amp != null && amp.Area > maxArea)
                        {
                            maxArea = amp.Area;
                            largest = face;
                            largestPlane = p;
                        }
                    }
                }
                catch { }
            }
            if (largest == null)
                throw new Exception("no twin pair and no planar face — not a recognizable plate");

            Vector3d fallbackNormal = largestPlane.Normal;
            if (!fallbackNormal.Unitize()) throw new Exception("largest-face normal degenerate");

            // Project every vertex onto the normal axis; extents define thickness.
            double minProj = double.PositiveInfinity;
            double maxProj = double.NegativeInfinity;
            foreach (var v in brep.Vertices)
            {
                var d = v.Location - largestPlane.Origin;
                double proj = d.X * fallbackNormal.X + d.Y * fallbackNormal.Y + d.Z * fallbackNormal.Z;
                if (proj < minProj) minProj = proj;
                if (proj > maxProj) maxProj = proj;
            }
            double fallbackThickness = maxProj - minProj;
            if (fallbackThickness < tol)
                throw new Exception("plate thickness ≤ tolerance (fallback path)");

            double midProj = (minProj + maxProj) * 0.5;
            var fallbackMidOrigin = largestPlane.Origin + fallbackNormal * midProj;
            var fallbackMidPlane = new Plane(fallbackMidOrigin, fallbackNormal);

            return new PlateInfo
            {
                MidPlane = fallbackMidPlane,
                Normal = fallbackNormal,
                Thickness = fallbackThickness,
            };
        }

        /// <summary>
        /// Section a plate Brep with its mid-plane via Intersection.BrepPlane.
        /// Returns the centerline outline curves (typically one closed curve
        /// for a simple plate, possibly more for plates with holes).
        /// </summary>
        private static Curve[] SectionPlateAtMidPlane(Brep brep, Plane midPlane, double tol)
        {
            if (!Intersection.BrepPlane(brep, midPlane, tol, out Curve[] curves, out _))
                return Array.Empty<Curve>();
            return curves ?? Array.Empty<Curve>();
        }

        /// <summary>
        /// Intersect the joint line with each outline curve; return the
        /// topmost or bottommost hit (along <paramref name="upDir"/>).
        /// </summary>
        private static Point3d FindExtremeJointHit(
            Curve[] outline, Line jointLine, Vector3d upDir, bool findTop, double tol)
        {
            // CurveLine expects a Line that's effectively infinite for the test.
            // Extend the joint line generously to ensure we don't miss hits.
            var bbox = BoundingBox.Empty;
            foreach (var c in outline) if (c != null) bbox.Union(c.GetBoundingBox(true));
            double extend = bbox.Diagonal.Length + 1.0;
            var extPt0 = jointLine.From - jointLine.Direction * extend;
            var extPt1 = jointLine.To + jointLine.Direction * extend;
            var extLineCurve = new LineCurve(extPt0, extPt1);

            bool any = false;
            double bestScore = findTop ? double.NegativeInfinity : double.PositiveInfinity;
            Point3d bestHit = Point3d.Origin;

            foreach (var curve in outline)
            {
                if (curve == null) continue;
                var ci = Intersection.CurveCurve(curve, extLineCurve, tol, tol);
                if (ci == null) continue;
                foreach (var evt in ci)
                {
                    var pt = evt.PointA;
                    double score = pt.X * upDir.X + pt.Y * upDir.Y + pt.Z * upDir.Z;
                    if (findTop)
                    {
                        if (score > bestScore) { bestScore = score; bestHit = pt; any = true; }
                    }
                    else
                    {
                        if (score < bestScore) { bestScore = score; bestHit = pt; any = true; }
                    }
                }
            }

            if (!any) throw new Exception(
                findTop ? "joint line missed plate outline (top)" : "joint line missed plate outline (bottom)");
            return bestHit;
        }

        /// <summary>
        /// Distance along <paramref name="dir"/> from <paramref name="a"/> to
        /// <paramref name="b"/>. Absolute value — direction-agnostic.
        /// </summary>
        private static double DistanceAlong(Point3d a, Point3d b, Vector3d dir)
        {
            return Math.Abs((b.X - a.X) * dir.X + (b.Y - a.Y) * dir.Y + (b.Z - a.Z) * dir.Z);
        }

        // ---------------------------------------------------------------
        // Cutter builders
        // ---------------------------------------------------------------

        /// <summary>
        /// Build the frame's compound cutter set: a rectangular slot
        /// extending upward from the bottom anchor (so the boolean removes
        /// material along the joint where the stringer passes through the
        /// frame), plus a full-circle rat-hole cylinder centered ON the
        /// frame's bottom edge (the half inside the plate body becomes the
        /// rat-hole bite; the half outside is geometrically harmless).
        ///
        /// Both cutters extrude along the frame's normal by cutterDepth
        /// centered on the mid-plane, so they punch cleanly through the
        /// frame's full thickness.
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
            // cutterDepth so the cutter cleanly overcuts past the bottom
            // (joins the rat-hole cylinder's overcut region without leaving
            // residual material at the very edge).
            //
            // Phase 20a.7.8 — frame slot is a stadium with chord at the
            // joint center (Y = slotHeight) and dome extending PAST the
            // chord into the stringer's body (Y = slotHeight + r). The
            // rectangular portion's length is exactly `slotHeight` so the
            // chord on the cut face aligns precisely with the joint
            // midpoint. The dome is a cavity inside the stringer's solid
            // material — it can't collide with the stringer's downward
            // dome (which is itself a cavity inside the frame's material).
            var slotPlane = MakeSafeCutPlane(anchor, frameWidthDir, upDir);
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
                    var slotCutter = ExtrudeClosedPlanarCurve(poly, frameNormal, cutterDepth);
                    if (slotCutter != null) cutters.Add(slotCutter);
                }
            }

            // Full-circle rat hole at the bottom anchor — rotationally
            // symmetric, sidesteps half-circle orientation issues. Cylinder
            // axis = frame normal; the half inside the plate body becomes
            // the rat hole, the half outside does nothing under boolean
            // subtraction.
            if (ratHoleRadius > tol)
            {
                var ratHoleCircleFrame = new Plane(anchor, frameNormal);
                var circle = new Circle(ratHoleCircleFrame, ratHoleRadius);
                var cyl = new Cylinder(circle, cutterDepth);
                var cylBrep = cyl.ToBrep(true, true);
                if (cylBrep != null && cylBrep.IsValid)
                {
                    cylBrep.Translate(frameNormal * (-cutterDepth / 2.0));
                    EnsureOutwardOrientation(cylBrep);
                    cutters.Add(cylBrep);
                }
            }

            return cutters.ToArray();
        }

        /// <summary>
        /// Build the stringer's stadium cutter. Open end at the stringer's
        /// top anchor (where the frame enters from above); rounded bottom at
        /// the joint midpoint. After boolean subtraction the stringer has a
        /// stadium-shaped notch opening from its top edge.
        ///
        /// Width and length are final-dimension values from the caller (no
        /// kerf compensation — CAM handles that at toolpath time). The
        /// stadium extrudes along the stringer's normal by cutterDepth
        /// centered on the mid-plane.
        /// </summary>
        private static Brep[] BuildStringerStadiumCutter(
            Point3d anchor,
            Vector3d stringerNormal,
            Vector3d stringerWidthDir,
            Vector3d upDir,
            double slotWidth,
            double slotDepth,
            double cutterDepth,
            double tol)
        {
            double r = slotWidth / 2.0;
            double L = slotDepth;

            // Stadium plane: X=stringerWidthDir (lateral), Y=upDir (along length).
            // Stadium extends DOWN from the anchor.
            //
            // Phase 20a.7.8 — chord lands at Y=-L (the joint center), with
            // the dome extending PAST the chord into the frame's body
            // (apex at Y=-(L+r)). The rectangular portion's length is
            // exactly L so the chord on the cut face aligns precisely with
            // the joint midpoint, matching the frame's upward chord.
            //
            // Top side overcuts UP past the stringer's top edge by
            // cutterDepth (Phase 20a.7.6) so the cutter cleanly punches
            // through and leaves the slot open at the top.
            var slotPlane = MakeSafeCutPlane(anchor, stringerWidthDir, upDir);

            //   p0 = (-r, +cutterDepth)  top-left, above stringer top edge (overcut)
            //   p1 = (-r, -L)            left chord endpoint at joint center
            //   arcMid = (0, -(L + r))   apex of half-circle BEYOND the chord
            //   p2 = (+r, -L)            right chord endpoint at joint center
            //   p3 = (+r, +cutterDepth)  top-right, above stringer top edge
            //   close p3 → p0            top edge (above stringer)
            var p0 = slotPlane.PointAt(-r, +cutterDepth);
            var p1 = slotPlane.PointAt(-r, -L);
            var arcMid = slotPlane.PointAt(0, -(L + r));
            var p2 = slotPlane.PointAt(+r, -L);
            var p3 = slotPlane.PointAt(+r, +cutterDepth);

            var leftSide = new LineCurve(p0, p1);
            var bottomArc = new ArcCurve(new Arc(p1, arcMid, p2));
            if (!bottomArc.IsValid) return Array.Empty<Brep>();
            var rightSide = new LineCurve(p2, p3);
            var top = new LineCurve(p3, p0);

            var poly = new PolyCurve();
            poly.Append(leftSide);
            poly.Append(bottomArc);
            poly.Append(rightSide);
            poly.Append(top);
            if (!poly.MakeClosed(0.001)) return Array.Empty<Brep>();

            var cutter = ExtrudeClosedPlanarCurve(poly, stringerNormal, cutterDepth);
            return cutter != null ? new[] { cutter } : Array.Empty<Brep>();
        }

        // ---------------------------------------------------------------
        // Curve / plane / extrusion helpers
        // ---------------------------------------------------------------

        /// <summary>
        /// Build a plane with explicit X / Y axes, both unitized. Right-handed:
        /// the plane normal = xAxis × yAxis. Used so the cutter helpers don't
        /// depend on curve-winding-derived plane normals (which flip the
        /// extrusion direction unpredictably).
        /// </summary>
        private static Plane MakeSafeCutPlane(Point3d origin, Vector3d xAxis, Vector3d yAxis)
        {
            var x = xAxis;
            if (!x.Unitize()) x = Vector3d.XAxis;
            var y = yAxis;
            if (!y.Unitize()) y = Vector3d.YAxis;
            return new Plane(origin, x, y);
        }

        /// <summary>
        /// Extrude a closed planar curve perpendicular to its plane along
        /// <paramref name="extrudeDir"/> by <paramref name="depth"/>, then
        /// translate by −extrudeDir × depth/2 so the resulting Brep is
        /// centered on the original curve's plane.
        ///
        /// Uses Surface.CreateExtrusion (direction-explicit, winding-agnostic)
        /// rather than Extrusion.Create (which derives direction from curve
        /// winding). Caps planar ends via CapPlanarHoles so the result is a
        /// closed manifold suitable for boolean operations.
        /// </summary>
        private static Brep ExtrudeClosedPlanarCurve(Curve profile, Vector3d extrudeDir, double depth)
        {
            if (profile == null) return null;
            var dir = extrudeDir;
            if (!dir.Unitize()) return null;
            var srf = Surface.CreateExtrusion(profile, dir * depth);
            if (srf == null) return null;
            var brep = srf.ToBrep();
            if (brep == null) return null;
            brep = brep.CapPlanarHoles(0.001);
            if (brep == null || !brep.IsValid) return null;
            brep.Translate(dir * (-depth / 2.0));
            // Phase 20a.7.5: ensure outward-facing normals so the cutter
            // reads as solid (not void) to Rhino's boolean algorithm. See
            // EnsureOutwardOrientation for why.
            EnsureOutwardOrientation(brep);
            return brep;
        }

        /// <summary>
        /// Phase 20a.7.5 — verify the cutter Brep is a closed solid with
        /// outward-pointing face normals, and Flip if inward-pointing.
        ///
        /// Rhino's CreateBooleanDifference treats a cutter with INWARD
        /// normals as describing the void OUTSIDE the cutter's volume —
        /// equivalent to subtracting "everything not the cutter" from the
        /// part. Result: only the cutter-intersect-part sliver survives,
        /// matching the "Δ=204" symptom (cutter ≈ 0.93 in³ but cut removed
        /// the entire 204 in³ plate, leaving the 0.234 in³ overlap).
        ///
        /// Surface.CreateExtrusion + CapPlanarHoles can produce inward
        /// orientation depending on the profile curve's winding direction
        /// vs. the extrusion direction. Cylinder.ToBrep can do the same.
        /// SolidOrientation == Outward is the safe state for cutters.
        /// </summary>
        private static void EnsureOutwardOrientation(Brep brep)
        {
            if (brep == null) return;
            try
            {
                if (brep.SolidOrientation == BrepSolidOrientation.Inward)
                    brep.Flip();
            }
            catch { }
        }

        // ---------------------------------------------------------------
        // Boolean application
        // ---------------------------------------------------------------

        /// <summary>
        /// Apply a list of accumulated cutters to a single part via one
        /// Brep.CreateBooleanDifference call. Returns the largest-volume
        /// result Brep on success, or null on failure.
        ///
        /// Preconditions (enforced by the caller's silent-skip): cutters is
        /// non-null and Count &gt; 0.
        /// </summary>
        private static Brep ApplyCutters(Brep part, IReadOnlyList<Brep> cutters, double tol)
        {
            if (part == null || cutters == null || cutters.Count == 0) return null;
            try
            {
                var result = Brep.CreateBooleanDifference(
                    new[] { part }, cutters.ToArray(), tol, false);
                if (result == null || result.Length == 0) return null;
                return result.Where(b => b != null && b.IsValid)
                             .OrderByDescending(GetVolume)
                             .FirstOrDefault();
            }
            catch { return null; }
        }

        private static double GetVolume(Brep brep)
        {
            try { var vp = VolumeMassProperties.Compute(brep); if (vp != null) return vp.Volume; } catch { }
            try { return brep.GetBoundingBox(true).Diagonal.Length; } catch { }
            return 0;
        }
    }
}
