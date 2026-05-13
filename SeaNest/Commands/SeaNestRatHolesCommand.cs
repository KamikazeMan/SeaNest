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

        // Nominal clearance between plate and member slot, per side. Final slot
        // width = plateThickness + 2 × SlotClearancePerSide; drawn slot width
        // shrinks from there by kerf. Independent of cutting method.
        private const double SlotClearancePerSide = 1.0 / 32.0;   // 1/32" each side

        // Phase 20a — reject crossings less than this angle from coplanar (i.e.
        // less than 30° between plate face and member face). Beyond that the
        // slot would need >2× nominal length to clear the oblique cross-section
        // and physical assembly becomes impractical.
        private const double SkewRejectAngleDegrees = 30.0;

        // Length filter for raw intersection curves before any cutter math —
        // drops tangent-contact zero-or-near-zero segments.
        private const double IntersectionLengthFilterFactor = 10.0;

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
            // Phase 20a: SeaNest pre-compensates for kerf. Drawn cut path is
            // inset by kerf/2 from each side so the actual cut lands at nominal
            // dimensions. Default Plasma — most common for steel marine work.
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

            // --- Debug markers ---
            var gd = new GetOption();
            gd.SetCommandPrompt("Show debug markers at each intersection?");
            int optYes = gd.AddOption("Yes");
            int optNo = gd.AddOption("No");
            gd.SetDefaultString("No");
            gd.AcceptNothing(true);
            bool showDebug = false;
            if (gd.Get() == GetResult.Option && gd.Option().Index == optYes)
                showDebug = true;

            RhinoApp.WriteLine(string.Format(
                "SeaNest Rat Holes: radius {0:G3} {1}, method {2} (kerf {3:G3} {1}).",
                radius, unitLabel, methodName, kerf));

            // --- Select plates ---
            // Phase 20a: vocabulary aligned with Phase 19b (plates + structural
            // members) instead of the prior FRAMES/STRINGERS labels which
            // inverted standard marine usage.
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
            // ID-based self-pick filter — same pattern Phase 19b's SeaNestNest
            // member prompt uses. Plates re-picked here are silently filtered
            // (with a count-summary message) so a fat-finger on selection
            // doesn't produce self-intersection garbage.
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

            int debugLayerIdx = GetOrCreateLayer(doc, "SeaNest_DebugMarkers", System.Drawing.Color.Red);

            // --- Per-intersection compute: accumulate cutters per part ---
            // Cutters accumulated in lists indexed by source-part index. After
            // the full intersection pass we apply ONE Brep.CreateBooleanDifference
            // per part with all its accumulated cutters — far more reliable than
            // sequential per-cut booleans (the original code's fallback path
            // already establishes this pattern).
            var plateCutters = new List<List<Brep>>();
            var memberCutters = new List<List<Brep>>();
            for (int i = 0; i < plates.Count; i++) plateCutters.Add(new List<Brep>());
            for (int i = 0; i < members.Count; i++) memberCutters.Add(new List<Brep>());

            var debugPoints = new List<Point3d>();
            int skewSkipped = 0;
            int kerfTooLargeSkipped = 0;
            int edgeTooShortSkipped = 0;
            double minLength = modelTol * IntersectionLengthFilterFactor;

            for (int p = 0; p < plates.Count; p++)
            {
                var plate = plates[p];
                var plateBBox = plate.GetBoundingBox(true);
                // Phase 20a.1: plate thickness is needed for slot-width math
                // and cutter depths, but NOT a rejection criterion. Plate
                // thickness and rat-hole radius are on different geometric
                // axes (radius is in-plane along the edge; thickness is
                // perpendicular to the face). Thin plates with reasonable
                // radii are valid. Only kerf-compensated drawn dimensions
                // and edge-segment length are tested per-intersection below.
                var plateThickness = MinBBoxAxis(plateBBox);

                // Identify plate's main (largest-area) face for the plate-side
                // anchor projection. Same heuristic the original Phase 2 code
                // used; preserves anchor-finding behavior across the rewrite.
                BrepFace mainFace = FindLargestFace(plate);
                if (mainFace == null)
                {
                    RhinoApp.WriteLine(string.Format("  Plate {0}: no main face found — skipped.", p + 1));
                    continue;
                }

                Plane facePlane;
                if (!mainFace.TryGetPlane(out facePlane, modelTol * 10.0))
                {
                    var midU = mainFace.Domain(0).Mid;
                    var midV = mainFace.Domain(1).Mid;
                    var midPt = mainFace.PointAt(midU, midV);
                    var n = mainFace.NormalAt(midU, midV);
                    facePlane = new Plane(midPt, n);
                }
                Vector3d plateNormal = facePlane.Normal;
                if (!plateNormal.Unitize()) plateNormal = Vector3d.ZAxis;

                // Project world -Z onto the plate plane to get a "down" direction
                // for the anchor-at-lower-end heuristic. Carried over from the
                // original anchor logic — rat holes go on the lower edge of the
                // plate where members bottom out.
                Vector3d faceDown = -Vector3d.ZAxis;
                faceDown = faceDown - (faceDown * plateNormal) * plateNormal;
                if (!faceDown.Unitize()) faceDown = -facePlane.YAxis;

                double dedupeTol = Math.Max(modelTol * 5.0, radius * 0.5);
                var anchorsThisPlate = new List<Point3d>();

                for (int m = 0; m < members.Count; m++)
                {
                    var member = members[m];
                    var memberBBox = member.GetBoundingBox(true);
                    var overlap = BoundingBox.Intersection(plateBBox, memberBBox);
                    if (!overlap.IsValid) continue;

                    // Phase 20a.1: member thickness sizes the cutter depth (4×
                    // through-cut) but is NOT a rejection criterion. Member
                    // thickness is along the plate-normal direction (slot
                    // depth); the rat-hole radius is on the plate face along
                    // the edge. They're geometrically unrelated. Real
                    // feasibility is checked per-intersection after kerf
                    // compensation below.
                    var memberThickness = MinBBoxAxis(memberBBox);

                    Curve[] curves;
                    bool ok;
                    try
                    {
                        ok = Intersection.BrepBrep(plate, member, modelTol, out curves, out _);
                    }
                    catch (Exception ex)
                    {
                        RhinoApp.WriteLine(string.Format("  BrepBrep error: {0}", ex.Message));
                        continue;
                    }
                    if (!ok || curves == null || curves.Length == 0) continue;

                    var joined = Curve.JoinCurves(curves, modelTol);
                    var loops = (joined != null && joined.Length > 0) ? joined : curves;

                    Vector3d memberNormal = ComputeMemberNormal(member);

                    foreach (var loop in loops)
                    {
                        if (loop == null || !loop.IsValid) continue;
                        if (loop.GetLength() < minLength) continue;

                        var anchor = FindEdgeAnchor(loop, mainFace, faceDown, radius);
                        if (anchor == null) continue;
                        var anchorPt = anchor.Value.Anchor;
                        var alongPlateEdge = anchor.Value.AlongEdge;

                        // Dedupe within the same plate (different member loops can
                        // produce the same anchor for adjacent members).
                        bool dup = anchorsThisPlate.Any(a => a.DistanceTo(anchorPt) <= dedupeTol);
                        if (dup) continue;
                        anchorsThisPlate.Add(anchorPt);
                        if (showDebug) debugPoints.Add(anchorPt);

                        // Member-side direction: vector between two farthest sample
                        // points on the loop is "along the plate edge through the
                        // member" — the slot extends INTO the member from the edge
                        // along this axis. Orient it pointing AWAY from the plate
                        // edge anchor (into the member body) using the loop's bbox
                        // center as reference.
                        Vector3d alongMember;
                        if (!ComputeAlongMember(loop, anchorPt, out alongMember)) continue;

                        // Skew angle: angle between plate face normal and member
                        // face normal. For perfectly perpendicular faces, |n·n| = 0
                        // → asin(0) = 0° "from perpendicular". For coplanar faces,
                        // |n·n| = 1 → asin(1) = 90° "from perpendicular".
                        double skewCos = Math.Abs(plateNormal * memberNormal);
                        if (skewCos > 1.0) skewCos = 1.0;
                        double skewAngleFromPerpDeg = Math.Asin(skewCos) * 180.0 / Math.PI;
                        if (skewAngleFromPerpDeg > (90.0 - SkewRejectAngleDegrees))
                        {
                            RhinoApp.WriteLine(string.Format(
                                "  Skew too steep ({0:F1}° from perpendicular) on plate {1} × member {2} — skipped.",
                                skewAngleFromPerpDeg, p + 1, m + 1));
                            skewSkipped++;
                            continue;
                        }

                        // Skew length compensation: angle between faces is
                        // (90° - skewAngleFromPerp), so sin(angleBetween) =
                        // cos(skewAngleFromPerp). Slot length grows by
                        // 1/sin(angleBetween) so the oblique footprint clears.
                        double sinAngleBetween = Math.Cos(skewAngleFromPerpDeg * Math.PI / 180.0);
                        if (sinAngleBetween < 0.5) sinAngleBetween = 0.5; // belt+suspenders

                        // Slot dimensions: nominal first, then kerf-compensated.
                        // Nominal width = plate thickness + per-side clearance.
                        // Drawn width = nominal − kerf (kerf removes width during cut).
                        double nominalSlotWidth = plateThickness + 2.0 * SlotClearancePerSide;
                        double drawnSlotWidth = nominalSlotWidth - kerf;
                        if (drawnSlotWidth < modelTol)
                        {
                            RhinoApp.WriteLine(string.Format(
                                "  Drawn slot width {0:G3} ≤ 0 after kerf comp (kerf {1:G3} ≥ nominal {2:G3}) — skipped.",
                                drawnSlotWidth, kerf, nominalSlotWidth));
                            kerfTooLargeSkipped++;
                            continue;
                        }

                        // Phase 20a.5: stadium length = plate's cross-section
                        // depth in the slot direction. The slot only needs to
                        // clear the plate's thickness (plus clearance each
                        // side); for oblique crossings the plate's projection
                        // along alongMember grows by 1/sin(angleBetween).
                        //
                        // The pre-20a.5 formula was loop.GetLength()/sinAngle,
                        // which conflated the perpendicular contact width with
                        // the slot depth and produced 12-25"+ slot lengths for
                        // 6"-tall members.
                        double nominalSlotLength = (plateThickness + 2.0 * SlotClearancePerSide) / sinAngleBetween;
                        double drawnSlotLength = nominalSlotLength - kerf;
                        if (drawnSlotLength < drawnSlotWidth)
                            drawnSlotLength = drawnSlotWidth;   // stadium length must accommodate the round end

                        // Plate-side rat hole: kerf comp shrinks the half-circle's
                        // drawn radius by kerf/2 (kerf removes material on each
                        // side of the cut path).
                        double drawnRadius = radius - kerf / 2.0;
                        if (drawnRadius < modelTol)
                        {
                            RhinoApp.WriteLine(string.Format(
                                "  Drawn rat-hole radius {0:G3} ≤ 0 after kerf comp — skipped.",
                                drawnRadius));
                            kerfTooLargeSkipped++;
                            continue;
                        }

                        // Phase 20a.1 — edge-segment-fits check. The plate's
                        // edge at this intersection must be long enough to
                        // accept the half-circle's chord (2 × drawnRadius).
                        // Approximated by the intersection-curve length, which
                        // is the plate edge segment in contact with the member.
                        // Independent of plate thickness, member thickness, or
                        // skew angle.
                        if (loop.GetLength() < 2.0 * drawnRadius)
                        {
                            RhinoApp.WriteLine(string.Format(
                                "  Plate {0} × member {1}: edge segment {2:G3} < 2 × drawn radius {3:G3} — skipped.",
                                p + 1, m + 1, loop.GetLength(), 2.0 * drawnRadius));
                            edgeTooShortSkipped++;
                            continue;
                        }

                        var plateCutter = BuildHalfCircleCutter(
                            anchorPt, plateNormal, alongPlateEdge, plateBBox.Center,
                            drawnRadius, plateThickness * 4.0);
                        if (plateCutter == null) continue;

                        Vector3d acrossMember = Vector3d.CrossProduct(alongMember, memberNormal);
                        if (!acrossMember.Unitize()) continue;

                        var memberCutter = BuildStadiumCutter(
                            anchorPt, alongMember, acrossMember, memberNormal,
                            drawnSlotWidth, drawnSlotLength, memberThickness * 4.0);
                        if (memberCutter == null) continue;

                        // Phase 20a.4 — TEMPORARY orientation-vector diagnostic.
                        // Logs the axes feeding into the cutter helpers plus
                        // sanity-check dot products: plateInto = (plateCentroid
                        // - anchorPt) · plateNormal. Positive means plateNormal
                        // already points INTO the plate body (good for the
                        // current translation logic); negative means it points
                        // OUTWARD and the cutter will end up on the wrong side.
                        // Same for memberInto with alongMember and the member
                        // centroid. Strip in Phase 20a.5 once the orientation
                        // bug is identified.
                        var plateCentroid = plateBBox.Center;
                        var memberCentroid = memberBBox.Center;
                        double plateIntoDot = (plateCentroid - anchorPt) * plateNormal;
                        double memberIntoDot = (memberCentroid - anchorPt) * alongMember;
                        RhinoApp.WriteLine(string.Format(
                            "[ratHole-diag] Plate {0} × Member {1} anchor=({2:F2},{3:F2},{4:F2}), " +
                            "plateThickness={5:F3}, memberThickness={6:F3}.",
                            p + 1, m + 1, anchorPt.X, anchorPt.Y, anchorPt.Z,
                            plateThickness, memberThickness));
                        RhinoApp.WriteLine(string.Format(
                            "[ratHole-diag]   plateNormal=({0:F3},{1:F3},{2:F3}), " +
                            "alongPlateEdge=({3:F3},{4:F3},{5:F3}), " +
                            "plateCentroid=({6:F2},{7:F2},{8:F2}), plateInto·n={9:F3} " +
                            "({10}).",
                            plateNormal.X, plateNormal.Y, plateNormal.Z,
                            alongPlateEdge.X, alongPlateEdge.Y, alongPlateEdge.Z,
                            plateCentroid.X, plateCentroid.Y, plateCentroid.Z,
                            plateIntoDot,
                            plateIntoDot > 0 ? "plateNormal points INTO body" : "plateNormal points OUTWARD"));
                        RhinoApp.WriteLine(string.Format(
                            "[ratHole-diag]   alongMember=({0:F3},{1:F3},{2:F3}), " +
                            "acrossMember=({3:F3},{4:F3},{5:F3}), " +
                            "memberNormal=({6:F3},{7:F3},{8:F3}), " +
                            "memberCentroid=({9:F2},{10:F2},{11:F2}), memberInto·a={12:F3} " +
                            "({13}).",
                            alongMember.X, alongMember.Y, alongMember.Z,
                            acrossMember.X, acrossMember.Y, acrossMember.Z,
                            memberNormal.X, memberNormal.Y, memberNormal.Z,
                            memberCentroid.X, memberCentroid.Y, memberCentroid.Z,
                            memberIntoDot,
                            memberIntoDot > 0 ? "alongMember points INTO body" : "alongMember points OUTWARD"));
                        RhinoApp.WriteLine(string.Format(
                            "[ratHole-diag]   drawnRadius={0:F3}, drawnSlotW={1:F3}, drawnSlotL={2:F3}, " +
                            "cutterDepthPlate={3:F3}, cutterDepthMember={4:F3}.",
                            drawnRadius, drawnSlotWidth, drawnSlotLength,
                            plateThickness * 4.0, memberThickness * 4.0));

                        plateCutters[p].Add(plateCutter);
                        memberCutters[m].Add(memberCutter);
                    }
                }
            }

            if (showDebug && debugPoints.Count > 0)
            {
                var debugAttr = new ObjectAttributes { LayerIndex = debugLayerIdx };
                foreach (var pt in debugPoints) doc.Objects.AddPoint(pt, debugAttr);
            }

            // --- Apply accumulated cuts ---
            // Per Q1 confirmation, do NOT change the modified Brep's layer —
            // doc.Objects.Replace preserves the existing object's attributes.
            // The user's plates and members stay on whatever layer they were on.
            int platesCut = 0;
            int totalPlateCuts = 0;
            int membersCut = 0;
            int totalMemberCuts = 0;
            int booleanFailures = 0;

            for (int p = 0; p < plates.Count; p++)
            {
                // Phase 20a.2 — silent-skip when no cutters were accumulated
                // for this plate. Reasons cutters may be zero:
                //   (a) no member's bbox overlapped this plate, or
                //   (b) every per-pair intersection loop was rejected by a
                //       feasibility check (edge-too-short, kerf-too-large,
                //       skew-too-steep, drawnRadius/drawnSlotWidth ≤ 0).
                // Per-rejection messages were already logged at the rejection
                // site; the apply pass doesn't repeat itself. Silent-skip is
                // correct: the plate stays unmodified, which IS the right
                // outcome when nothing needs cutting. CreateBooleanDifference
                // is NEVER called with an empty cutter list.
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
                // Phase 20a.2 — same silent-skip contract as the plate loop
                // above. CreateBooleanDifference never receives an empty
                // cutter list.
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
                "  SeaNest Rat Holes: {0} plate(s) cut with {1} rat hole(s); " +
                "{2} member(s) cut with {3} slot(s).",
                platesCut, totalPlateCuts, membersCut, totalMemberCuts));
            if (skewSkipped > 0)
                RhinoApp.WriteLine(string.Format(
                    "  Skipped {0} intersection(s) for skew > {1}° from perpendicular.",
                    skewSkipped, 90 - SkewRejectAngleDegrees));
            if (kerfTooLargeSkipped > 0)
                RhinoApp.WriteLine(string.Format(
                    "  Skipped {0} intersection(s) — kerf too large (drawn dimension ≤ 0).",
                    kerfTooLargeSkipped));
            if (edgeTooShortSkipped > 0)
                RhinoApp.WriteLine(string.Format(
                    "  Skipped {0} intersection(s) — edge segment too short for rat-hole radius.",
                    edgeTooShortSkipped));
            if (booleanFailures > 0)
                RhinoApp.WriteLine(string.Format(
                    "  {0} boolean failure(s) — affected parts left unchanged.", booleanFailures));
            RhinoApp.WriteLine("==========================================");

            doc.Views.Redraw();
            return Result.Success;
        }

        // ----- helpers -----

        private static double MinBBoxAxis(BoundingBox bb)
        {
            var d = bb.Diagonal;
            return Math.Min(d.X, Math.Min(d.Y, d.Z));
        }

        private static BrepFace FindLargestFace(Brep brep)
        {
            BrepFace main = null;
            double maxArea = 0;
            foreach (var face in brep.Faces)
            {
                try
                {
                    using (var fb = face.DuplicateFace(false))
                    {
                        var amp = AreaMassProperties.Compute(fb);
                        if (amp != null && amp.Area > maxArea)
                        {
                            maxArea = amp.Area;
                            main = face;
                        }
                    }
                }
                catch { }
            }
            return main;
        }

        private static Vector3d ComputeMemberNormal(Brep member)
        {
            var face = FindLargestFace(member);
            if (face == null) return Vector3d.ZAxis;
            var n = face.NormalAt(face.Domain(0).Mid, face.Domain(1).Mid);
            if (!n.Unitize()) return Vector3d.ZAxis;
            return n;
        }

        /// <summary>
        /// Find the rat-hole anchor on a plate × member intersection loop.
        /// Anchor is the centroid of points near the loop's "lower end" along
        /// the plate's faceDown direction, projected onto the plate's main
        /// face. Also returns the unit vector along the plate edge at that
        /// point (perpendicular to faceDown in the plate plane).
        ///
        /// Carried over from the original Phase 2 anchor logic with light
        /// refactoring; the math is unchanged.
        /// </summary>
        private static (Point3d Anchor, Vector3d AlongEdge)? FindEdgeAnchor(
            Curve loop, BrepFace mainFace, Vector3d faceDown, double radius)
        {
            const int samples = 200;
            var pts = new List<Point3d>();
            for (int k = 0; k <= samples; k++)
            {
                if (!loop.NormalizedLengthParameter((double)k / samples, out double t)) continue;
                pts.Add(loop.PointAt(t));
            }
            if (pts.Count < 2) return null;

            Point3d centroid;
            try
            {
                var amp = AreaMassProperties.Compute(loop);
                centroid = amp != null ? amp.Centroid : loop.GetBoundingBox(true).Center;
            }
            catch { centroid = loop.GetBoundingBox(true).Center; }

            double maxScore = double.NegativeInfinity;
            foreach (var pt in pts)
            {
                double s = (pt - centroid) * faceDown;
                if (s > maxScore) maxScore = s;
            }
            var lowerEnd = pts.Where(pt =>
                Math.Abs((pt - centroid) * faceDown - maxScore) < radius * 0.5).ToList();
            if (lowerEnd.Count == 0) lowerEnd = pts;

            Point3d anchor = new Point3d(0, 0, 0);
            foreach (var pt in lowerEnd) anchor += (Vector3d)pt;
            anchor /= lowerEnd.Count;

            if (!mainFace.ClosestPoint(anchor, out double u, out double v)) return null;
            anchor = mainFace.PointAt(u, v);

            var n = mainFace.NormalAt(u, v);
            if (!n.Unitize()) return null;
            // alongEdge orientation matters: with alongEdge = n × faceDown,
            // BuildHalfCircleCutter's cross(n, alongEdge) yields -faceDown
            // (provably: n × (n × v) = -v for unit n perpendicular to v),
            // which is the "anti-down" direction the half-circle should bulge
            // into. cross(faceDown, n) would give the opposite orientation
            // and the cutter would bulge OUT of the plate edge.
            var alongEdge = Vector3d.CrossProduct(n, faceDown);
            if (!alongEdge.Unitize()) alongEdge = Vector3d.XAxis;

            return (anchor, alongEdge);
        }

        /// <summary>
        /// Compute the "into the member" axis for the slot. Uses the two
        /// farthest sample points on the intersection loop (their separation
        /// vector approximates the loop's principal direction), then orients
        /// the result to point AWAY from the plate edge anchor.
        /// </summary>
        private static bool ComputeAlongMember(Curve loop, Point3d anchorPt, out Vector3d alongMember)
        {
            alongMember = Vector3d.Zero;
            const int samples = 64;
            var pts = new List<Point3d>();
            for (int k = 0; k <= samples; k++)
            {
                if (!loop.NormalizedLengthParameter((double)k / samples, out double t)) continue;
                pts.Add(loop.PointAt(t));
            }
            if (pts.Count < 2) return false;

            double maxDist = 0;
            Point3d a0 = pts[0], a1 = pts[0];
            for (int i = 0; i < pts.Count; i++)
            {
                for (int j = i + 1; j < pts.Count; j++)
                {
                    double d = pts[i].DistanceTo(pts[j]);
                    if (d > maxDist) { maxDist = d; a0 = pts[i]; a1 = pts[j]; }
                }
            }
            var v = a1 - a0;
            if (!v.Unitize()) return false;

            var loopBBoxCenter = loop.GetBoundingBox(true).Center;
            if ((loopBBoxCenter - anchorPt) * v < 0) v = -v;
            alongMember = v;
            return true;
        }

        /// <summary>
        /// Half-circle cutter for the plate edge. Closed planar curve
        /// (semicircular arc + closing chord), extruded perpendicular to the
        /// plate by cutterDepth, then centered on the plate face via
        /// CenterAlongAxis so the cutter passes cleanly through the plate's
        /// full thickness.
        ///
        /// The chord lies along alongPlateEdge at the edge anchor. The arc
        /// bulges INTO the plate (perpendicular to alongPlateEdge in the plate
        /// plane), so after boolean subtraction the plate has a half-circle
        /// bite taken out of its edge — the rat hole.
        ///
        /// Drawn radius is already kerf-compensated by the caller
        /// (drawnRadius = userRadius − kerf/2).
        ///
        /// Phase 20a.6: <paramref name="plateBodyReference"/> (typically the
        /// plate's bbox centroid) is used to verify intoPlate's sign. The
        /// cross product <c>plateNormal × alongPlateEdge</c> produces a
        /// perpendicular but its sign depends on alongPlateEdge's orientation,
        /// which the caller computes via <c>n × faceDown</c> — not guaranteed
        /// to point INTO the plate body. We dot-product against
        /// (bodyReference − anchor) and flip when needed. Without this, the
        /// half-circle bulges AWAY from the plate body and the rat hole bites
        /// into empty space.
        /// </summary>
        private static Brep BuildHalfCircleCutter(
            Point3d edgeAnchor,
            Vector3d plateNormal,
            Vector3d alongPlateEdge,
            Point3d plateBodyReference,
            double drawnRadius,
            double cutterDepth)
        {
            try
            {
                // intoPlate = direction in the plate plane perpendicular to
                // alongPlateEdge. Cross product gives a perpendicular — sign
                // depends on input orientation, so we verify and flip below.
                var intoPlate = Vector3d.CrossProduct(plateNormal, alongPlateEdge);
                if (!intoPlate.Unitize()) return null;

                // Phase 20a.6: sign-correct so intoPlate points TOWARD the
                // plate body. Same winding-agnostic pattern as CenterAlongAxis
                // — don't trust the construction's sign, verify against an
                // actual body reference.
                if ((plateBodyReference - edgeAnchor) * intoPlate < 0)
                    intoPlate = -intoPlate;

                // Slot plane: X=alongPlateEdge, Y=intoPlate, origin=edgeAnchor.
                // Half-circle: arc from (-r, 0) through (0, r) to (+r, 0)
                // (180° sweep into +Y), then closing chord from (+r, 0) back
                // to (-r, 0) along Y=0 (the plate edge).
                var slotPlane = new Plane(edgeAnchor, alongPlateEdge, intoPlate);
                double r = drawnRadius;
                var p0 = slotPlane.PointAt(-r, 0);
                var p1 = slotPlane.PointAt(+r, 0);
                var arcMid = slotPlane.PointAt(0, r);

                var arc = new Arc(p0, arcMid, p1);
                if (!arc.IsValid) return null;
                var arcCurve = new ArcCurve(arc);
                var chord = new LineCurve(p1, p0);

                var poly = new PolyCurve();
                poly.Append(arcCurve);
                poly.Append(chord);
                if (!poly.MakeClosed(0.001)) return null;

                var extrusion = Extrusion.Create(poly, cutterDepth, true);
                if (extrusion == null) return null;
                var brep = extrusion.ToBrep();
                if (brep == null || !brep.IsValid) return null;
                // Phase 20a.5: bbox-based centering (replaces blind
                // Translate(plateNormal * -cutterDepth/2)). Extrusion.Create
                // extrudes in the profile curve's normal direction, which can
                // be +slotPlane.Normal or -slotPlane.Normal depending on the
                // curve's winding (CW vs CCW from the +normal view). The
                // half-circle curve here winds CW from +plateNormal, so
                // Extrusion goes -plateNormal — the prior fixed-sign
                // translation then double-shifted the cutter off the plate.
                // Centering by bbox is winding-agnostic.
                CenterAlongAxis(brep, edgeAnchor, plateNormal);
                return brep;
            }
            catch { return null; }
        }

        /// <summary>
        /// Stadium cutter for the member. Closed planar curve (line + 180°
        /// arc + line + closing line), extruded perpendicular to the member
        /// by cutterDepth, then translated by -cutterDepth/2 along the member
        /// normal so the cutter is centered on the member face.
        ///
        /// Curve geometry (in slot-local frame X=acrossMember, Y=alongMember,
        /// origin=edgeAnchor on the member edge):
        ///   p0 = (-r, 0)        bottom-left, on the member edge
        ///   p1 = (-r, L-r)      where the rounded top begins
        ///   arcMid = (0, L)     apex of the rounded top
        ///   p2 = (+r, L-r)      where the rounded top ends
        ///   p3 = (+r, 0)        bottom-right, on the member edge
        ///   bottom edge p3→p0   closes the figure across the member edge
        ///
        /// After boolean subtraction the member has a stadium-shaped notch:
        /// rounded at the deep end, open at the member edge — exactly what
        /// the plate's tab passes through.
        ///
        /// Drawn width and length are already kerf-compensated by the caller.
        /// </summary>
        private static Brep BuildStadiumCutter(
            Point3d edgeAnchor,
            Vector3d alongMember,
            Vector3d acrossMember,
            Vector3d throughMember,
            double drawnWidth,
            double drawnLength,
            double cutterDepth)
        {
            try
            {
                double r = drawnWidth / 2.0;
                double L = drawnLength;
                if (L <= 2 * r) L = 2 * r + 1e-4;   // length must accommodate the arc

                var slotPlane = new Plane(edgeAnchor, acrossMember, alongMember);
                var p0 = slotPlane.PointAt(-r, 0);
                var p1 = slotPlane.PointAt(-r, L - r);
                var arcMid = slotPlane.PointAt(0, L);
                var p2 = slotPlane.PointAt(+r, L - r);
                var p3 = slotPlane.PointAt(+r, 0);

                var leftSide = new LineCurve(p0, p1);
                var topArc = new ArcCurve(new Arc(p1, arcMid, p2));
                if (!topArc.IsValid) return null;
                var rightSide = new LineCurve(p2, p3);
                var bottom = new LineCurve(p3, p0);

                var poly = new PolyCurve();
                poly.Append(leftSide);
                poly.Append(topArc);
                poly.Append(rightSide);
                poly.Append(bottom);
                if (!poly.MakeClosed(0.001)) return null;

                var extrusion = Extrusion.Create(poly, cutterDepth, true);
                if (extrusion == null) return null;
                var brep = extrusion.ToBrep();
                if (brep == null || !brep.IsValid) return null;
                // Phase 20a.5: bbox-based centering, same fix as
                // BuildHalfCircleCutter. Stadium curve winds CCW from
                // +throughMember view (opposite of the half-circle's CW
                // winding), so this happens to land correctly with the prior
                // fixed-sign translation, but using CenterAlongAxis here too
                // makes the helper winding-agnostic for future curve changes.
                CenterAlongAxis(brep, edgeAnchor, throughMember);
                return brep;
            }
            catch { return null; }
        }

        /// <summary>
        /// Phase 20a.5 — center a cutter's bounding box on
        /// <paramref name="anchor"/>'s projection along
        /// <paramref name="centerAxis"/>. Robust against Extrusion.Create
        /// extruding in either +normal or -normal direction (curve winding
        /// determines this). In-plane axes are unaffected — only the
        /// extrusion-axis position changes.
        /// </summary>
        private static void CenterAlongAxis(Brep cutter, Point3d anchor, Vector3d centerAxis)
        {
            if (cutter == null) return;
            var c = cutter.GetBoundingBox(true).Center;
            double anchorProj = anchor.X * centerAxis.X
                              + anchor.Y * centerAxis.Y
                              + anchor.Z * centerAxis.Z;
            double centerProj = c.X * centerAxis.X
                              + c.Y * centerAxis.Y
                              + c.Z * centerAxis.Z;
            cutter.Translate(centerAxis * (anchorProj - centerProj));
        }

        /// <summary>
        /// Apply a list of accumulated cutters to a single part via one
        /// Brep.CreateBooleanDifference call. Returns the largest-volume
        /// result Brep on success, or null on failure.
        ///
        /// Preconditions (enforced by the caller's silent-skip): cutters is
        /// non-null and Count &gt; 0. Defensive null-and-empty check inside
        /// is kept as a belt-and-suspenders safety against future refactors.
        ///
        /// Phase 20a.3 — TEMPORARY deep diagnostic. Logs per-cutter validity,
        /// solidity, volume, bbox, centroid containment, and a part-vs-cutter
        /// intersection probe; logs the multi-cutter result; on failure runs
        /// a sequential single-cutter fallback so each cutter's behavior is
        /// isolated. Strip in Phase 20a.4 once the failure mode is identified.
        /// </summary>
        private static Brep ApplyCutters(
            Brep part, IReadOnlyList<Brep> cutters, double tol, string label)
        {
            if (part == null || cutters == null || cutters.Count == 0) return null;

            // Per-part info
            var partBBox = part.GetBoundingBox(true);
            double partVol = GetVolume(part);
            RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: part IsValid={1}, IsSolid={2}, bbox=[{3:F2}×{4:F2}×{5:F2}], vol={6:G3}, tol={7:G3}.",
                label, part.IsValid, part.IsSolid,
                partBBox.Diagonal.X, partBBox.Diagonal.Y, partBBox.Diagonal.Z,
                partVol, tol));

            // Per-cutter probe
            for (int i = 0; i < cutters.Count; i++)
            {
                var c = cutters[i];
                if (c == null)
                {
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: NULL.", label, i + 1));
                    continue;
                }
                var cb = c.GetBoundingBox(true);
                double cvol = GetVolume(c);
                var centroid = cb.Center;
                bool centroidInside = false;
                try { centroidInside = part.IsPointInside(centroid, tol, false); }
                catch (Exception ex)
                {
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: IsPointInside threw: {2}",
                        label, i + 1, ex.Message));
                }

                RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0} cutter {1}: IsValid={2}, IsSolid={3}, vol={4:G3}, " +
                    "bbox=[{5:F3}×{6:F3}×{7:F3}], centroid=({8:F2},{9:F2},{10:F2}), " +
                    "centroidInsidePart={11}.",
                    label, i + 1, c.IsValid, c.IsSolid, cvol,
                    cb.Diagonal.X, cb.Diagonal.Y, cb.Diagonal.Z,
                    centroid.X, centroid.Y, centroid.Z,
                    centroidInside));

                // Probe: does part actually intersect cutter?
                try
                {
                    bool intOk = Rhino.Geometry.Intersect.Intersection.BrepBrep(
                        part, c, tol, out Curve[] iCurves, out _);
                    int n = iCurves == null ? 0 : iCurves.Length;
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: BrepBrep(part, cutter) ok={2}, curves={3}.",
                        label, i + 1, intOk, n));
                }
                catch (Exception ex)
                {
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: BrepBrep probe threw: {2}",
                        label, i + 1, ex.Message));
                }
            }

            // Multi-cutter attempt
            Brep[] result;
            try
            {
                result = Brep.CreateBooleanDifference(
                    new[] { part }, cutters.ToArray(), tol, false);
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0}: multi-cutter CreateBooleanDifference threw: {1}",
                    label, ex.Message));
                result = null;
            }

            int resultLen = result == null ? -1 : result.Length;
            RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: multi-cutter CreateBooleanDifference → {1}.",
                label, result == null ? "null" : (resultLen + " result(s)")));

            if (result != null && resultLen > 0)
            {
                var best = result.Where(b => b != null && b.IsValid)
                                 .OrderByDescending(GetVolume)
                                 .FirstOrDefault();
                if (best != null)
                {
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0}: multi-cutter OK, best result vol={1:G3}.",
                        label, GetVolume(best)));
                    return best;
                }
            }

            // Fallback diagnostic: sequential single-cutter
            RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: multi-cutter failed; trying sequential single-cutter cuts...",
                label));

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
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: sequential cut threw: {2}",
                        label, i + 1, ex.Message));
                    seq = null;
                }
                if (seq == null || seq.Length == 0)
                {
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: sequential cut FAILED ({2}).",
                        label, i + 1, seq == null ? "null" : "empty array"));
                    continue;
                }
                var seqBest = seq.Where(b => b != null && b.IsValid)
                                 .OrderByDescending(GetVolume)
                                 .FirstOrDefault();
                if (seqBest == null)
                {
                    RhinoApp.WriteLine(string.Format(
                        "[ratHole-diag] {0} cutter {1}: sequential cut returned {2} brep(s), none valid.",
                        label, i + 1, seq.Length));
                    continue;
                }
                double newVol = GetVolume(seqBest);
                double oldVol = GetVolume(current);
                RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0} cutter {1}: sequential cut OK, vol {2:G3} → {3:G3} (Δ={4:G3}).",
                    label, i + 1, oldVol, newVol, oldVol - newVol));
                current = seqBest;
                seqOk++;
            }

            if (seqOk > 0)
            {
                RhinoApp.WriteLine(string.Format(
                    "[ratHole-diag] {0}: sequential fallback applied {1}/{2} cut(s) successfully — RETURNING the partial result.",
                    label, seqOk, cutters.Count));
                return current;
            }

            RhinoApp.WriteLine(string.Format(
                "[ratHole-diag] {0}: ALL sequential cuts failed too — returning null.", label));
            return null;
        }

        private static int GetOrCreateLayer(RhinoDoc doc, string name, System.Drawing.Color color)
        {
            int idx = doc.Layers.FindByFullPath(name, -1);
            if (idx < 0) { var l = new Layer { Name = name, Color = color }; idx = doc.Layers.Add(l); }
            return idx;
        }

        private static double GetVolume(Brep brep)
        {
            try { var vp = VolumeMassProperties.Compute(brep); if (vp != null) return vp.Volume; } catch { }
            try { return brep.GetBoundingBox(true).Diagonal.Length; } catch { }
            return 0;
        }
    }
}
