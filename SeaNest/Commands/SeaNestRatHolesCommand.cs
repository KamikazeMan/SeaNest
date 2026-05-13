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
                var result = ApplyCutters(plates[p], plateCutters[p], modelTol, string.Format("Plate {0}", p + 1));
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
                var result = ApplyCutters(members[m], memberCutters[m], modelTol, string.Format("Member {0}", m + 1));
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

            // Phase 20a.7.1 temporary diagnostic — confirms which path
            // (twin-pair vs fallback) was taken for each plate. Strip in
            // Phase 20a.7.2 once verified for the user's test geometry.
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] frame plate info: midPlane normal=({0:F3},{1:F3},{2:F3}), thickness={3:G3}.",
                frameInfo.Normal.X, frameInfo.Normal.Y, frameInfo.Normal.Z, frameInfo.Thickness));
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] stringer plate info: midPlane normal=({0:F3},{1:F3},{2:F3}), thickness={3:G3}.",
                stringerInfo.Normal.X, stringerInfo.Normal.Y, stringerInfo.Normal.Z, stringerInfo.Thickness));

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

            // Cut heights along the joint centerline.
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

            // Rectangle slot, anchor at the bottom-midpoint, extending upward.
            var slotPlane = MakeSafeCutPlane(anchor, frameWidthDir, upDir);

            // Phase 20a.7.2 temporary diagnostic — strip in 20a.7.3.
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] frame cutter at anchor=({0:F2},{1:F2},{2:F2}), slotWidth={3:G3}, slotHeight={4:G3}, ratHoleRadius={5:G3}, cutterDepth={6:G3}.",
                anchor.X, anchor.Y, anchor.Z, slotWidth, slotHeight, ratHoleRadius, cutterDepth));
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag]   frameNormal=({0:F3},{1:F3},{2:F3}), frameWidthDir=({3:F3},{4:F3},{5:F3}), upDir=({6:F3},{7:F3},{8:F3}).",
                frameNormal.X, frameNormal.Y, frameNormal.Z,
                frameWidthDir.X, frameWidthDir.Y, frameWidthDir.Z,
                upDir.X, upDir.Y, upDir.Z));
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag]   slotPlane.Origin=({0:F2},{1:F2},{2:F2}), XAxis=({3:F3},{4:F3},{5:F3}), YAxis=({6:F3},{7:F3},{8:F3}), ZAxis=({9:F3},{10:F3},{11:F3}).",
                slotPlane.Origin.X, slotPlane.Origin.Y, slotPlane.Origin.Z,
                slotPlane.XAxis.X, slotPlane.XAxis.Y, slotPlane.XAxis.Z,
                slotPlane.YAxis.X, slotPlane.YAxis.Y, slotPlane.YAxis.Z,
                slotPlane.ZAxis.X, slotPlane.ZAxis.Y, slotPlane.ZAxis.Z));

            var rect = MakeRectangleCurve(slotPlane, slotWidth, slotHeight);
            if (rect != null)
            {
                var slotCutter = ExtrudeClosedPlanarCurve(rect, frameNormal, cutterDepth);
                if (slotCutter != null)
                {
                    var bb = slotCutter.GetBoundingBox(true);
                    Rhino.RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag]   slot cutter bbox=[{0:F3}×{1:F3}×{2:F3}], centroid=({3:F2},{4:F2},{5:F2}).",
                        bb.Diagonal.X, bb.Diagonal.Y, bb.Diagonal.Z,
                        bb.Center.X, bb.Center.Y, bb.Center.Z));
                    cutters.Add(slotCutter);
                }
                else
                {
                    Rhino.RhinoApp.WriteLine("[ratHole-diag]   slot cutter EXTRUSION FAILED.");
                }
            }
            else
            {
                Rhino.RhinoApp.WriteLine("[ratHole-diag]   slot rect curve build FAILED.");
            }

            // Full-circle rat hole at the bottom anchor — sidesteps half-circle
            // sign-orientation issues. Cylinder axis = frame normal; the half
            // inside the plate body becomes the rat hole, the half outside
            // does nothing under boolean subtraction.
            if (ratHoleRadius > tol)
            {
                var ratHoleCircleFrame = new Plane(anchor, frameNormal);
                var circle = new Circle(ratHoleCircleFrame, ratHoleRadius);
                var cyl = new Cylinder(circle, cutterDepth);
                var cylBrep = cyl.ToBrep(true, true);
                if (cylBrep != null && cylBrep.IsValid)
                {
                    cylBrep.Translate(frameNormal * (-cutterDepth / 2.0));
                    var bb = cylBrep.GetBoundingBox(true);
                    Rhino.RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag]   full-circle cutter bbox=[{0:F3}×{1:F3}×{2:F3}], centroid=({3:F2},{4:F2},{5:F2}).",
                        bb.Diagonal.X, bb.Diagonal.Y, bb.Diagonal.Z,
                        bb.Center.X, bb.Center.Y, bb.Center.Z));
                    cutters.Add(cylBrep);
                }
                else
                {
                    Rhino.RhinoApp.WriteLine("[ratHole-diag]   full-circle cutter BUILD FAILED.");
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
            if (L < 2 * r) L = 2 * r + 1e-4;   // length must accommodate the arc end

            // Stadium plane: X=stringerWidthDir (lateral), Y=upDir (along length).
            // We want stadium to extend DOWN from the anchor (anchor is the top of
            // the slot, rounded end is at the bottom). So local Y goes from 0
            // (at anchor) to -L (rounded end deep in stringer body).
            //
            // ... but Y is upDir which already points toward +Z. Going to -L on
            // Y means going DOWNWARD in world. That's what we want — the slot
            // opens at the stringer's TOP edge (anchor) and extends down into
            // the stringer body.
            var slotPlane = MakeSafeCutPlane(anchor, stringerWidthDir, upDir);

            // Phase 20a.7.2 temporary diagnostic — strip in 20a.7.3.
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] stringer stadium at anchor=({0:F2},{1:F2},{2:F2}), slotWidth={3:G3}, slotDepth(L)={4:G3}, cutterDepth={5:G3}.",
                anchor.X, anchor.Y, anchor.Z, slotWidth, L, cutterDepth));
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag]   stringerNormal=({0:F3},{1:F3},{2:F3}), stringerWidthDir=({3:F3},{4:F3},{5:F3}), upDir=({6:F3},{7:F3},{8:F3}).",
                stringerNormal.X, stringerNormal.Y, stringerNormal.Z,
                stringerWidthDir.X, stringerWidthDir.Y, stringerWidthDir.Z,
                upDir.X, upDir.Y, upDir.Z));
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag]   slotPlane.Origin=({0:F2},{1:F2},{2:F2}), XAxis=({3:F3},{4:F3},{5:F3}), YAxis=({6:F3},{7:F3},{8:F3}), ZAxis=({9:F3},{10:F3},{11:F3}).",
                slotPlane.Origin.X, slotPlane.Origin.Y, slotPlane.Origin.Z,
                slotPlane.XAxis.X, slotPlane.XAxis.Y, slotPlane.XAxis.Z,
                slotPlane.YAxis.X, slotPlane.YAxis.Y, slotPlane.YAxis.Z,
                slotPlane.ZAxis.X, slotPlane.ZAxis.Y, slotPlane.ZAxis.Z));

            //   p0 = (-r, 0)        top-left, on stringer top edge
            //   p1 = (-r, -(L-r))   where the rounded bottom begins
            //   arcMid = (0, -L)    apex of rounded bottom
            //   p2 = (+r, -(L-r))   where the rounded bottom ends
            //   p3 = (+r, 0)        top-right, on stringer top edge
            //   back to p0          closes across the open top
            var p0 = slotPlane.PointAt(-r, 0);
            var p1 = slotPlane.PointAt(-r, -(L - r));
            var arcMid = slotPlane.PointAt(0, -L);
            var p2 = slotPlane.PointAt(+r, -(L - r));
            var p3 = slotPlane.PointAt(+r, 0);

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
            if (cutter != null)
            {
                var bb = cutter.GetBoundingBox(true);
                Rhino.RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag]   stringer stadium cutter bbox=[{0:F3}×{1:F3}×{2:F3}], centroid=({3:F2},{4:F2},{5:F2}).",
                    bb.Diagonal.X, bb.Diagonal.Y, bb.Diagonal.Z,
                    bb.Center.X, bb.Center.Y, bb.Center.Z));
                return new[] { cutter };
            }
            Rhino.RhinoApp.WriteLine("[ratHole-diag]   stringer stadium EXTRUSION FAILED.");
            return Array.Empty<Brep>();
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
        /// Rectangle in plane-local coords: X from −width/2 to +width/2,
        /// Y from 0 to height. Returns a closed PolylineCurve.
        /// </summary>
        private static Curve MakeRectangleCurve(Plane plane, double width, double height)
        {
            double hw = width / 2.0;
            var p0 = plane.PointAt(-hw, 0);
            var p1 = plane.PointAt(+hw, 0);
            var p2 = plane.PointAt(+hw, height);
            var p3 = plane.PointAt(-hw, height);
            var poly = new Polyline { p0, p1, p2, p3, p0 };
            return poly.ToNurbsCurve();
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
            return brep;
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
        private static Brep ApplyCutters(
            Brep part, IReadOnlyList<Brep> cutters, double tol, string label)
        {
            if (part == null || cutters == null || cutters.Count == 0) return null;

            // Phase 20a.7.2 temporary diagnostic — per-cutter validity +
            // centroid-inside-part check + multi/sequential boolean attempts.
            // Strip in 20a.7.3 once the failure mode is identified.
            var partBBox = part.GetBoundingBox(true);
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: part IsValid={1}, IsSolid={2}, bbox=[{3:F2}×{4:F2}×{5:F2}], {6} cutter(s).",
                label, part.IsValid, part.IsSolid,
                partBBox.Diagonal.X, partBBox.Diagonal.Y, partBBox.Diagonal.Z,
                cutters.Count));

            for (int i = 0; i < cutters.Count; i++)
            {
                var c = cutters[i];
                if (c == null)
                {
                    Rhino.RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: NULL.", label, i + 1));
                    continue;
                }
                var bb = c.GetBoundingBox(true);
                bool centroidInside = false;
                try { centroidInside = part.IsPointInside(bb.Center, tol, false); }
                catch { }
                Rhino.RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0} cutter {1}: IsValid={2}, IsSolid={3}, bbox=[{4:F3}×{5:F3}×{6:F3}], centroid=({7:F2},{8:F2},{9:F2}), centroidInsidePart={10}.",
                    label, i + 1, c.IsValid, c.IsSolid,
                    bb.Diagonal.X, bb.Diagonal.Y, bb.Diagonal.Z,
                    bb.Center.X, bb.Center.Y, bb.Center.Z, centroidInside));
            }

            // Multi-cutter attempt
            Brep[] multi;
            try
            {
                multi = Brep.CreateBooleanDifference(
                    new[] { part }, cutters.ToArray(), tol, false);
            }
            catch (Exception ex)
            {
                Rhino.RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0}: multi-cutter CreateBooleanDifference threw: {1}", label, ex.Message));
                multi = null;
            }
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: multi-cutter → {1}.",
                label, multi == null ? "null" : (multi.Length + " result(s)")));

            if (multi != null && multi.Length > 0)
            {
                var best = multi.Where(b => b != null && b.IsValid)
                                .OrderByDescending(GetVolume)
                                .FirstOrDefault();
                if (best != null) return best;
            }

            // Sequential fallback — apply cutters one at a time so we can tell
            // which one broke the multi-cutter pipeline.
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: multi-cutter failed; trying sequential cuts.", label));
            Brep current = part.DuplicateBrep();
            int seqOk = 0;
            for (int i = 0; i < cutters.Count; i++)
            {
                Brep[] seq;
                try
                {
                    seq = Brep.CreateBooleanDifference(
                        new[] { current }, new[] { cutters[i] }, tol, false);
                }
                catch (Exception ex)
                {
                    Rhino.RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: sequential threw: {2}", label, i + 1, ex.Message));
                    seq = null;
                }
                if (seq == null || seq.Length == 0)
                {
                    Rhino.RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: sequential FAILED.", label, i + 1));
                    continue;
                }
                var seqBest = seq.Where(b => b != null && b.IsValid)
                                 .OrderByDescending(GetVolume)
                                 .FirstOrDefault();
                if (seqBest == null)
                {
                    Rhino.RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: sequential returned {2} brep(s), none valid.", label, i + 1, seq.Length));
                    continue;
                }
                double oldV = GetVolume(current);
                double newV = GetVolume(seqBest);
                Rhino.RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0} cutter {1}: sequential OK, vol {2:G3} → {3:G3} (Δ={4:G3}).",
                    label, i + 1, oldV, newV, oldV - newV));
                current = seqBest;
                seqOk++;
            }
            if (seqOk > 0)
            {
                Rhino.RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0}: sequential applied {1}/{2} cut(s) — returning partial result.",
                    label, seqOk, cutters.Count));
                return current;
            }
            Rhino.RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: ALL sequential cuts failed.", label));
            return null;
        }

        private static double GetVolume(Brep brep)
        {
            try { var vp = VolumeMassProperties.Compute(brep); if (vp != null) return vp.Volume; } catch { }
            try { return brep.GetBoundingBox(true).Diagonal.Length; } catch { }
            return 0;
        }
    }
}
