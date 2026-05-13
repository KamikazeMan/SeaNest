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

        // Phase 20a — kerf compensation: SeaNest draws cuts smaller than nominal
        // by the cutter's kerf so the cutter's swept material removal lands the
        // final dimension on-target. Values are inches; for metric docs they're
        // scaled at the call site via Inches→ModelUnit conversion.
        private const double KerfRouter   = 0.250;     // 1/4"
        private const double KerfWaterjet = 0.0625;    // 1/16"
        private const double KerfPlasma   = 0.125;     // 1/8"

        // Total clearance between plate and slot. Final slot width =
        // matingThickness + ClearanceTotal; drawn slot width = final − kerf.
        private const double ClearanceTotal = 1.0 / 16.0;   // 1/16" total

        // Reject crossings less than this angle between plate and member faces
        // (i.e. when sin(angleBetween) < 0.05 ≈ 3°). Below this the slot
        // dimensions blow up and the joint is impractical to assemble.
        private const double ParallelRejectSinThreshold = 0.05;

        // Cutter extrusion depth multiplier. Cutter passes through plate's
        // thickness by this factor on each side (so 2× total). Comfortably
        // overcuts so the boolean cleans up reliably.
        private const double CutterOvercutFactor = 4.0;

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

            // --- Cutting method (determines kerf) ---
            var goMethod = new GetOption();
            goMethod.SetCommandPrompt("Cutting method");
            int optRouter = goMethod.AddOption("Router");
            int optWaterjet = goMethod.AddOption("Waterjet");
            int optPlasma = goMethod.AddOption("Plasma");
            goMethod.SetDefaultString("Plasma");
            goMethod.AcceptNothing(true);
            double kerf = KerfPlasma * inchScale;
            string methodName = "Plasma";
            if (goMethod.Get() == GetResult.Option)
            {
                int idx = goMethod.Option().Index;
                if      (idx == optRouter)   { kerf = KerfRouter   * inchScale; methodName = "Router"; }
                else if (idx == optWaterjet) { kerf = KerfWaterjet * inchScale; methodName = "Waterjet"; }
                else if (idx == optPlasma)   { kerf = KerfPlasma   * inchScale; methodName = "Plasma"; }
            }

            double clearance = ClearanceTotal * inchScale;

            RhinoApp.WriteLine(string.Format(
                "SeaNest Rat Holes: radius {0:G3} {1}, method {2} (kerf {3:G3} {1}), clearance {4:G3} {1}.",
                radius, unitLabel, methodName, kerf, clearance));

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
                            clearance, kerf, radius,
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
            double clearance, double kerf, double frameRatHoleRadius,
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

            // Cut heights along the joint centerline.
            double frameCutHeight = DistanceAlong(frameBottomAnchor, stringerMidPoint, upDir);
            double stringerSlotDepth = DistanceAlong(stringerMidPoint, stringerTopAnchor, upDir);

            if (frameCutHeight <= tol)
                throw new Exception("frame cut height ≤ 0");
            if (stringerSlotDepth <= tol)
                throw new Exception("stringer slot depth ≤ 0");

            // Slot widths: nominal = mating thickness + clearance, scaled by
            // 1/sinAngle for oblique crossings (the perpendicular plate thickness
            // projects wider along the slot's width axis), then minus kerf.
            double frameSlotWidth = (stringerInfo.Thickness + clearance) / sinAngle - kerf;
            double stringerSlotWidth = (frameInfo.Thickness + clearance) / sinAngle - kerf;

            if (frameSlotWidth <= tol)
                throw new Exception("frame slot width ≤ 0 after kerf comp");
            if (stringerSlotWidth <= tol)
                throw new Exception("stringer slot width ≤ 0 after kerf comp");

            // In-plane width axes (perpendicular to upDir, in each plate's plane).
            Vector3d frameWidthDir = Vector3d.CrossProduct(upDir, nf);
            if (!frameWidthDir.Unitize()) throw new Exception("frame width axis degenerate");
            Vector3d stringerWidthDir = Vector3d.CrossProduct(upDir, ns);
            if (!stringerWidthDir.Unitize()) throw new Exception("stringer width axis degenerate");

            double cutterDepth = Math.Max(frameInfo.Thickness, stringerInfo.Thickness)
                                 * CutterOvercutFactor;

            Brep[] frameCutters = BuildFrameCompoundCutters(
                frameBottomAnchor, nf, frameWidthDir, upDir,
                frameSlotWidth, frameCutHeight, frameRatHoleRadius,
                cutterDepth, tol);

            Brep[] stringerCutters = BuildStringerStadiumCutter(
                stringerTopAnchor, ns, stringerWidthDir, upDir,
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
        /// Identify a thin plate's mid-plane by finding the two largest
        /// anti-parallel planar faces. Mid-plane origin is the midpoint of
        /// the two face origins; normal averages them. Thickness is the
        /// perpendicular separation between the two faces.
        ///
        /// For Phase 20a.7 simplicity, falls through on parallel-twin-not-found.
        /// Curved hull plates (where the "twin" faces aren't quite planar) could
        /// be handled by integrating Phase 14.1.2's FindTwinPlateFace from
        /// BrepFlattener.cs in a follow-up if a real workflow surfaces.
        /// </summary>
        private static PlateInfo GetPlateInfoFromLargestPlanarFaces(Brep brep, double tol)
        {
            // Largest planar face.
            BrepFace mainFace = null;
            Plane mainPlane = Plane.Unset;
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
                            mainFace = face;
                            mainPlane = p;
                        }
                    }
                }
                catch { }
            }
            if (mainFace == null)
                throw new Exception("no planar face on this Brep");

            // Anti-parallel twin face (dot product near -1).
            BrepFace twinFace = null;
            Plane twinPlane = Plane.Unset;
            double bestAntiparallel = -0.9;   // require ≤ -0.9 (≈ < 26° off antiparallel)
            foreach (var face in brep.Faces)
            {
                if (face.FaceIndex == mainFace.FaceIndex) continue;
                if (!face.IsPlanar(tol * 10.0)) continue;
                if (!face.TryGetPlane(out Plane p, tol * 10.0)) continue;
                double dot = p.Normal * mainPlane.Normal;
                if (dot < bestAntiparallel)
                {
                    bestAntiparallel = dot;
                    twinFace = face;
                    twinPlane = p;
                }
            }
            if (twinFace == null)
                throw new Exception("no anti-parallel twin face — plate isn't a thin sheet");

            // Mid-plane: average origin (projected onto the normal axis), normal = main normal.
            Vector3d normal = mainPlane.Normal;
            if (!normal.Unitize()) throw new Exception("plate normal degenerate");
            Point3d midOrigin = new Point3d(
                (mainPlane.Origin.X + twinPlane.Origin.X) * 0.5,
                (mainPlane.Origin.Y + twinPlane.Origin.Y) * 0.5,
                (mainPlane.Origin.Z + twinPlane.Origin.Z) * 0.5);
            var midPlane = new Plane(midOrigin, normal);

            // Thickness = absolute perpendicular separation between the two face planes.
            double thickness = Math.Abs((twinPlane.Origin - mainPlane.Origin) * normal);
            if (thickness < tol)
                throw new Exception("plate thickness ≤ tolerance");

            return new PlateInfo
            {
                MidPlane = midPlane,
                Normal = normal,
                Thickness = thickness,
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
            var rect = MakeRectangleCurve(slotPlane, slotWidth, slotHeight);
            if (rect != null)
            {
                var slotCutter = ExtrudeClosedPlanarCurve(rect, frameNormal, cutterDepth);
                if (slotCutter != null) cutters.Add(slotCutter);
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
                    // Cylinder extends from circle plane along axis by cutterDepth.
                    // Translate by -axis*cutterDepth/2 to center on the mid-plane.
                    cylBrep.Translate(frameNormal * (-cutterDepth / 2.0));
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
        /// Width and length are pre-kerf-compensated by the caller. The
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
