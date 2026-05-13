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
    /// SeaNestSlots — symmetric tab-and-slot joinery for plate × member
    /// intersections. BOTH parts get matching stadium-slot cuts that
    /// interlock during assembly; welded perimeters seal the joint
    /// (suitable for watertight applications).
    ///
    /// Phase 20b.1: same geometry algorithm as SeaNestRatHoles, minus
    /// the full-circle rat-hole cylinder. For each plate × member pair:
    ///   - Joint centerline := <c>Intersection.PlanePlane(midPlate, midMember)</c>
    ///   - Plate stadium: opens at plate's bottom edge, extends upward
    ///     to joint midpoint with dome cap past the chord
    ///   - Member stadium: opens at member's top edge, extends downward
    ///     to joint midpoint with dome cap past the chord
    ///   - Both built via <see cref="JointGeometryHelpers.BuildStadiumSlotCutter"/>
    ///
    /// Distinction from SeaNestRatHoles:
    ///   - SeaNestRatHoles: dual cut + rat-hole drainage. The plate's
    ///     cutter is a stadium PLUS a full-circle rat-hole cylinder at
    ///     the bottom edge for weld access / water drainage.
    ///   - SeaNestSlots (this command): dual cut, no rat-hole. Welded
    ///     perimeter on both sides seals the joint.
    /// </summary>
    public class SeaNestSlotsCommand : Command
    {
        public override string EnglishName => "SeaNestSlots";

        // Total clearance between mating thickness and slot width. Final
        // slot width = matingThickness + ClearanceTotal. No kerf
        // compensation — CAM handles that at toolpath time.
        private const double ClearanceTotal = 1.0 / 16.0;   // 1/16" total

        // Reject crossings less than this angle between plate and member
        // faces. Below this, slot dimensions blow up and the joint is
        // impractical to assemble.
        private const double ParallelRejectSinThreshold = 0.05;

        // Cutter extrusion depth multiplier. Cutter passes through the
        // part's thickness by this factor on each side (so 2× total).
        private const double CutterOvercutFactor = 2.0;

        // Offset the cutter centroid off the part's mid-plane by this
        // multiple of modelTol. Breaks face-coincidence between cutter
        // and mid-plane that would otherwise confuse Rhino's boolean
        // algorithm into returning the intersection sliver instead of
        // the part-minus-hole.
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
                "SeaNest Slots: clearance {0:G3} {1}. Both plates and members get interlocking stadium slots.",
                clearance, unitLabel));

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

                    Brep[] plateSlotCutter, memberSlotCutter;
                    try
                    {
                        BuildSymmetricSlotCutters(
                            plate, member, clearance, modelTol,
                            out plateSlotCutter, out memberSlotCutter);
                    }
                    catch (Exception ex)
                    {
                        RhinoApp.WriteLine(string.Format(
                            "  Plate {0} × Member {1}: {2} — skipped.",
                            p + 1, m + 1, ex.Message));
                        pairsSkipped++;
                        continue;
                    }

                    if (plateSlotCutter != null)  plateCutters[p].AddRange(plateSlotCutter);
                    if (memberSlotCutter != null) memberCutters[m].AddRange(memberSlotCutter);
                }
            }

            // --- Apply accumulated cuts to both plates AND members ---
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
                "  SeaNest Slots: {0} plate(s) cut with {1} slot(s); " +
                "{2} member(s) cut with {3} slot(s).",
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

        /// <summary>
        /// Per-pair joint-compute. Builds the symmetric pair of stadium
        /// slot cutters — one for the plate (opens at its bottom edge,
        /// extends up to joint midpoint) and one for the member (opens
        /// at its top edge, extends down to joint midpoint). Shared
        /// joint-geometry math (mid-plane detection, joint centerline,
        /// anchor finding) delegated to <see cref="JointGeometryHelpers"/>.
        ///
        /// Mirrors <c>SeaNestRatHolesCommand.BuildFrameStringerJointCutters</c>
        /// minus the rat-hole cylinder.
        /// </summary>
        private static void BuildSymmetricSlotCutters(
            Brep plate, Brep member,
            double clearance, double tol,
            out Brep[] plateCutters, out Brep[] memberCutters)
        {
            Vector3d worldUp = Vector3d.ZAxis;

            PlateInfo plateInfo = JointGeometryHelpers.GetPlateInfo(plate, tol);
            PlateInfo memberInfo = JointGeometryHelpers.GetPlateInfo(member, tol);

            Vector3d np = plateInfo.Normal; np.Unitize();
            Vector3d nm = memberInfo.Normal; nm.Unitize();

            double sinAngle = Vector3d.CrossProduct(np, nm).Length;
            if (sinAngle < ParallelRejectSinThreshold)
                throw new Exception("plate and member are nearly parallel");

            if (!Rhino.Geometry.Intersect.Intersection.PlanePlane(
                    plateInfo.MidPlane, memberInfo.MidPlane, out Line jointLine))
                throw new Exception("mid-planes do not intersect");

            Vector3d upDir = jointLine.Direction;
            if (!upDir.Unitize())
                throw new Exception("joint centerline has zero length");
            // Phase 20c.3: body-relative sign-correction toward the
            // member's centroid (intrinsic to the joint geometry, robust
            // for curved/tilted members). World-Z fallback when joint
            // midpoint coincides with member centroid (degenerate case).
            var jointMid = jointLine.PointAt(0.5);
            var memberCenter = member.GetBoundingBox(true).Center;
            var towardMember = memberCenter - jointMid;
            if (towardMember.Length < tol)
            {
                if (upDir * worldUp < 0.0) upDir.Reverse();
            }
            else
            {
                if (upDir * towardMember < 0.0) upDir.Reverse();
            }

            // Section both plates at their own mid-planes.
            Curve[] plateOutline = JointGeometryHelpers.SectionPlateAtMidPlane(plate, plateInfo.MidPlane, tol);
            Curve[] memberOutline = JointGeometryHelpers.SectionPlateAtMidPlane(member, memberInfo.MidPlane, tol);
            if (plateOutline.Length == 0)
                throw new Exception("plate mid-plane section returned no curves");
            if (memberOutline.Length == 0)
                throw new Exception("member mid-plane section returned no curves");

            // Anchors: where the joint line crosses each outline.
            // Phase 20b.2 — for watertight slot assembly, parts slide
            // together from opposite directions. The plate's slot opens
            // at its TOP edge (extends DOWN into the plate body); the
            // member's slot opens at its BOTTOM edge (extends UP into the
            // member body). They interlock at the member's mid-height.
            //
            // (Compare to SeaNestRatHoles, where the plate slot opens at
            // its bottom edge and extends up — that's the
            // assemble-from-below pattern. SeaNestSlots is
            // assemble-from-opposing-sides, which mates the two slots'
            // chords at the same elevation but with the parts approaching
            // each other.)
            // Phase 20c.1: anchor lookups fall back to BrepBrep when the
            // mid-plane outline misses. Phase 20c.2: fallback is filtered
            // by proximity to the joint line so curved-member
            // brush-contacts at distant positions don't produce spurious
            // cuts. Phase 20c.3: cutoff is 10× MAX thickness (was 5× sum).
            double maxJointDistance = Math.Max(plateInfo.Thickness, memberInfo.Thickness) * 10.0;

            Point3d plateTopAnchor =
                ResolveAnchorWithFallback(plateOutline, jointLine, upDir, findTop: true,
                    plate, member, "plate", "top", maxJointDistance, tol);
            Point3d memberTopAnchor =
                ResolveAnchorWithFallback(memberOutline, jointLine, upDir, findTop: true,
                    member, plate, "member", "top", maxJointDistance, tol);
            Point3d memberBottomAnchor =
                ResolveAnchorWithFallback(memberOutline, jointLine, upDir, findTop: false,
                    member, plate, "member", "bottom", maxJointDistance, tol);

            // The slot-meeting elevation is still the member's midpoint —
            // same as SeaNestRatHoles. The member is the part that gets
            // interrupted at mid-height; the plate cut just terminates
            // there so the two slots' chord lines meet.
            Point3d memberMidPoint = new Point3d(
                (memberTopAnchor.X + memberBottomAnchor.X) * 0.5,
                (memberTopAnchor.Y + memberBottomAnchor.Y) * 0.5,
                (memberTopAnchor.Z + memberBottomAnchor.Z) * 0.5);

            // Chord positions:
            //   - Plate slot extends DOWN from plateTopAnchor to memberMidPoint
            //   - Member slot extends UP from memberBottomAnchor to memberMidPoint
            double plateSlotDepth = JointGeometryHelpers.DistanceAlong(memberMidPoint, plateTopAnchor, upDir);
            double memberSlotDepth = JointGeometryHelpers.DistanceAlong(memberBottomAnchor, memberMidPoint, upDir);

            if (plateSlotDepth <= tol) throw new Exception("plate slot depth ≤ 0");
            if (memberSlotDepth <= tol) throw new Exception("member slot depth ≤ 0");

            // Slot widths: mating thickness + clearance, scaled by
            // 1/sinAngle for oblique crossings.
            double plateSlotWidth = (memberInfo.Thickness + clearance) / sinAngle;
            double memberSlotWidth = (plateInfo.Thickness + clearance) / sinAngle;

            if (plateSlotWidth <= tol) throw new Exception("plate slot width ≤ 0");
            if (memberSlotWidth <= tol) throw new Exception("member slot width ≤ 0");

            // In-plane width axes (perpendicular to upDir, in each plate's plane).
            Vector3d plateWidthDir = Vector3d.CrossProduct(upDir, np);
            if (!plateWidthDir.Unitize()) throw new Exception("plate width axis degenerate");
            Vector3d memberWidthDir = Vector3d.CrossProduct(upDir, nm);
            if (!memberWidthDir.Unitize()) throw new Exception("member width axis degenerate");

            double cutterDepth = Math.Max(plateInfo.Thickness, memberInfo.Thickness)
                                 * CutterOvercutFactor;

            // Offset each cutter's center off its part's mid-plane.
            double midPlaneOffset = tol * CutterMidPlaneOffsetFactor;
            Point3d plateAnchorForCut = plateTopAnchor + np * midPlaneOffset;
            Point3d memberAnchorForCut = memberBottomAnchor + nm * midPlaneOffset;

            // Both slots use the shared stadium primitive. Slot-extension
            // directions are FLIPPED from SeaNestRatHoles for the
            // watertight assembly pattern (parts approach from opposite
            // sides):
            //   - Plate: anchor on top edge, slot extends -upDir (down)
            //   - Member: anchor on bottom edge, slot extends +upDir (up)
            plateCutters = JointGeometryHelpers.BuildStadiumSlotCutter(
                plateAnchorForCut, np, plateWidthDir, -upDir,
                plateSlotWidth, plateSlotDepth,
                cutterDepth, tol);

            memberCutters = JointGeometryHelpers.BuildStadiumSlotCutter(
                memberAnchorForCut, nm, memberWidthDir, upDir,
                memberSlotWidth, memberSlotDepth,
                cutterDepth, tol);
        }

        /// <summary>
        /// Phase 20c.1 — resolve a joint anchor via the mid-plane outline
        /// primary; on miss, log + fall back to BrepBrep intersection
        /// between the two parts. Throws when both fail (caught by the
        /// per-pair loop which logs the skip with plate/member indices).
        ///
        /// <paramref name="anchorPart"/> is the part whose anchor we
        /// want; <paramref name="otherPart"/> is the part it touches.
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
