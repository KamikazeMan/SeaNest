using System;
using System.Collections.Generic;
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
    /// SeaNestSlots — watertight tab-and-slot joinery. Plates remain solid;
    /// members get a slot that the plate passes through. Used when the
    /// joint must be watertight (welded around the slot perimeter on the
    /// member side, with the plate sealing the slot via its uninterrupted
    /// face).
    ///
    /// Phase 20b — Reuses Phase 20a.7's mid-plane-intersection joint
    /// algorithm via <see cref="JointGeometryHelpers"/>. The geometry
    /// pipeline is identical to SeaNestRatHoles up through anchor finding;
    /// the only difference is that no frame cutter is built (plate stays
    /// solid) and no rat-hole circle is added (no drainage path on a
    /// watertight joint).
    ///
    /// Distinction from SeaNestRatHoles:
    ///   - SeaNestRatHoles: BOTH plate and member are cut. Plate gets a
    ///     stadium slot + half-circle rat-hole at its edge; member gets a
    ///     stadium slot. Used for drainage / non-watertight assembly.
    ///   - SeaNestSlots (this command): ONLY the member is cut, with a
    ///     stadium slot. Plate stays solid. Used for watertight assembly.
    /// </summary>
    public class SeaNestSlotsCommand : Command
    {
        public override string EnglishName => "SeaNestSlots";

        // Total clearance between plate and slot. Final slot width =
        // plateThickness + ClearanceTotal. No kerf compensation — CAM
        // handles that at toolpath time. Same value as SeaNestRatHoles
        // for consistent marine fit standards.
        private const double ClearanceTotal = 1.0 / 16.0;   // 1/16" total

        // Reject crossings less than this angle between plate and member
        // faces. Below this, slot dimensions blow up and the joint is
        // impractical to assemble.
        private const double ParallelRejectSinThreshold = 0.05;

        // Cutter extrusion depth multiplier. Cutter passes through the
        // member's thickness by this factor on each side (so 2× total).
        private const double CutterOvercutFactor = 2.0;

        // Offset the cutter centroid off the member's mid-plane by this
        // multiple of modelTol. Breaks face-coincidence between cutter
        // and member mid-plane that would otherwise confuse Rhino's
        // boolean algorithm into returning the intersection sliver
        // instead of the part-minus-hole.
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
            double clearance = ClearanceTotal * inchScale;

            RhinoApp.WriteLine(string.Format(
                "SeaNest Slots (watertight): clearance {0:G3} {1}. Plates remain solid; members get slots.",
                clearance, unitLabel));

            // --- Select plates (these stay solid; their thickness sizes the slot) ---
            var goPlates = new GetObject();
            goPlates.SetCommandPrompt("Select plates (stay solid; pass through member slots)");
            goPlates.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
            goPlates.GroupSelect = true;
            goPlates.SubObjectSelect = false;
            goPlates.EnablePreSelect(false, false);
            goPlates.GetMultiple(1, 0);
            if (goPlates.CommandResult() != Result.Success) return goPlates.CommandResult();

            var plates = new List<Brep>();
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
                    plateIdSet.Add(obj.ObjectId);
                }
            }

            // --- Select structural members (these get cut) ---
            var goMembers = new GetObject();
            goMembers.SetCommandPrompt("Select structural members (get slots)");
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

            // --- Per-pair joint compute: accumulate member cutters only ---
            var memberCutters = new List<List<Brep>>();
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

                    Brep[] cutters;
                    try
                    {
                        cutters = BuildMemberSlotCutter(plate, member, clearance, modelTol);
                    }
                    catch (Exception ex)
                    {
                        RhinoApp.WriteLine(string.Format(
                            "  Plate {0} × Member {1}: {2} — skipped.",
                            p + 1, m + 1, ex.Message));
                        pairsSkipped++;
                        continue;
                    }

                    if (cutters != null) memberCutters[m].AddRange(cutters);
                }
            }

            // --- Apply accumulated cuts to members only (plates untouched) ---
            int membersCut = 0;
            int totalMemberCuts = 0;
            int booleanFailures = 0;

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
                "  SeaNest Slots: {0} member(s) cut with {1} slot(s). Plates unchanged.",
                membersCut, totalMemberCuts));
            if (pairsSkipped > 0)
                RhinoApp.WriteLine(string.Format(
                    "  Skipped {0} pair(s) due to joint-geometry issues (see messages above).",
                    pairsSkipped));
            if (booleanFailures > 0)
                RhinoApp.WriteLine(string.Format(
                    "  {0} boolean failure(s) — affected members left unchanged.", booleanFailures));
            RhinoApp.WriteLine("==========================================");

            doc.Views.Redraw();
            return Result.Success;
        }

        /// <summary>
        /// Per-pair joint-compute for the watertight-slot case. Builds ONLY
        /// the member's stadium slot cutter — no plate cutter, no rat-hole
        /// circle. Shared joint-geometry math (mid-plane detection, joint
        /// centerline, anchor finding) delegated to <see cref="JointGeometryHelpers"/>.
        /// </summary>
        private static Brep[] BuildMemberSlotCutter(
            Brep plate, Brep member,
            double clearance, double tol)
        {
            Vector3d worldUp = Vector3d.ZAxis;

            PlateInfo plateInfo = JointGeometryHelpers.GetPlateInfo(plate, tol);
            PlateInfo memberInfo = JointGeometryHelpers.GetPlateInfo(member, tol);

            Vector3d np = plateInfo.Normal; np.Unitize();
            Vector3d nm = memberInfo.Normal; nm.Unitize();

            // Joint centerline: intersection of the two mid-planes.
            double sinAngle = Vector3d.CrossProduct(np, nm).Length;
            if (sinAngle < ParallelRejectSinThreshold)
                throw new Exception("plate and member are nearly parallel");

            if (!Rhino.Geometry.Intersect.Intersection.PlanePlane(
                    plateInfo.MidPlane, memberInfo.MidPlane, out Line jointLine))
                throw new Exception("mid-planes do not intersect");

            Vector3d upDir = jointLine.Direction;
            if (!upDir.Unitize())
                throw new Exception("joint centerline has zero length");
            if (upDir * worldUp < 0.0) upDir.Reverse();

            // Member's mid-plane outline.
            Curve[] memberOutline = JointGeometryHelpers.SectionPlateAtMidPlane(member, memberInfo.MidPlane, tol);
            if (memberOutline.Length == 0)
                throw new Exception("member mid-plane section returned no curves");

            // Anchors: top + bottom of where the joint line crosses the
            // member's mid-plane outline. The stadium opens at the top
            // anchor and extends down to the joint midpoint (between the
            // top and bottom anchors).
            Point3d memberTopAnchor =
                JointGeometryHelpers.FindExtremeJointHit(memberOutline, jointLine, upDir, findTop: true, tol);
            Point3d memberBottomAnchor =
                JointGeometryHelpers.FindExtremeJointHit(memberOutline, jointLine, upDir, findTop: false, tol);

            Point3d memberMidPoint = new Point3d(
                (memberTopAnchor.X + memberBottomAnchor.X) * 0.5,
                (memberTopAnchor.Y + memberBottomAnchor.Y) * 0.5,
                (memberTopAnchor.Z + memberBottomAnchor.Z) * 0.5);

            // Slot depth: top anchor down to the member's mid-point along
            // the joint centerline. For a watertight joint, the slot
            // extends to the half-depth of the member — the plate
            // engages this far into the member's body, with the rest of
            // the member solid to provide the welded seal area.
            double slotDepth = JointGeometryHelpers.DistanceAlong(memberMidPoint, memberTopAnchor, upDir);
            if (slotDepth <= tol)
                throw new Exception("member slot depth ≤ 0");

            // Slot width: plate thickness + clearance, scaled by 1/sinAngle
            // for oblique crossings (the perpendicular plate thickness
            // projects wider along the slot's width axis).
            double slotWidth = (plateInfo.Thickness + clearance) / sinAngle;
            if (slotWidth <= tol)
                throw new Exception("member slot width ≤ 0");

            // In-plane width axis (perpendicular to upDir, in the member's
            // mid-plane).
            Vector3d memberWidthDir = Vector3d.CrossProduct(upDir, nm);
            if (!memberWidthDir.Unitize()) throw new Exception("member width axis degenerate");

            double cutterDepth = memberInfo.Thickness * CutterOvercutFactor;

            // Offset the cutter centroid off the member's mid-plane to
            // break face-coincidence with the boolean algorithm.
            double midPlaneOffset = tol * CutterMidPlaneOffsetFactor;
            Point3d anchorForCut = memberTopAnchor + nm * midPlaneOffset;

            // Shared stadium slot cutter (same geometry the rat-holes
            // command uses for its member side — chord at Y=-slotDepth,
            // dome past the chord, top edge overcutting past the member's
            // top edge by cutterDepth).
            return JointGeometryHelpers.BuildStadiumSlotCutter(
                anchorForCut, nm, memberWidthDir, upDir,
                slotWidth, slotDepth,
                cutterDepth, tol);
        }
    }
}
