using System;
using System.Collections.Generic;
using System.Linq;
using Eto.Forms;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Nesting;
using SeaNest.Nesting.Core.Verification;
using SeaNest.RhinoAdapters;
using RhinoCommand = Rhino.Commands.Command;

namespace SeaNest.Commands
{
    /// <summary>
    /// Re-nest already-flat 2D curves (e.g. output from a previous SeaNestNest run, or
    /// curves from any other source). Skips Brep flattening entirely — the selected curves
    /// ARE the polygons.
    ///
    /// Useful for iterating on sheet size / margin / spacing / rotation parameters
    /// without re-cooking the Brep geometry (which is slow for curved parts that go through Squish).
    /// </summary>
    public class SeaNestReNestCommand : RhinoCommand
    {
        public override string EnglishName => "SeaNestReNest";

        private const double DefaultSheetWidthIn = 96.0;
        private const double DefaultSheetHeightIn = 48.0;
        private const double DefaultThicknessIn = 0.25;
        private const double DefaultMarginIn = 0.25;
        private const double DefaultSpacingIn = 0.25;

        // Phase 18 — match BrepFlattener's vertex cap so the engine never sees
        // a curve's raw chord-tessellation. Original Rhino Curve is preserved
        // separately for visual output (see outerCurvePerPart below).
        private const int MaxVertices = 500;
        private const double EscalationFactor = 2.0;
        private const int MaxEscalations = 3;

        protected override Rhino.Commands.Result RunCommand(RhinoDoc doc, Rhino.Commands.RunMode mode)
        {
            bool isMetric = doc.ModelUnitSystem == UnitSystem.Millimeters
                         || doc.ModelUnitSystem == UnitSystem.Centimeters
                         || doc.ModelUnitSystem == UnitSystem.Meters;

            double inToModel = isMetric
                ? RhinoMath.UnitScale(UnitSystem.Inches, doc.ModelUnitSystem)
                : 1.0;

            double sheetW, sheetH, sheetT, margin, spacing;
            RotationStep rotStep;

            if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet width", DefaultSheetWidthIn * inToModel, out sheetW)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet height", DefaultSheetHeightIn * inToModel, out sheetH)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet thickness", DefaultThicknessIn * inToModel, out sheetT)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Margin", DefaultMarginIn * inToModel, out margin)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Spacing", DefaultSpacingIn * inToModel, out spacing)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForRotation(out rotStep)) return Rhino.Commands.Result.Cancel;

            var go = new GetObject();
            go.SetCommandPrompt("Select curves and labels to re-nest");
            // Phase 18.2 — accept curves AND annotations in one selection.
            // Closed curves get partitioned into outer/inner via pairwise
            // PlanarClosedCurveRelationship; open curves get associated with
            // their containing outer via a midpoint test; TextEntity
            // annotations get associated via their anchor (PlainText
            // string → namesPerPart). Selection is authoritative — only
            // what the user picks gets processed, so re-nesting a subset
            // of an existing layout works without picking up unrelated
            // labels still on SeaNest_Labels.
            go.GeometryFilter = ObjectType.Curve | ObjectType.Annotation;
            go.GroupSelect = true;
            go.SubObjectSelect = false;
            go.GetMultiple(1, 0);
            if (go.CommandResult() != Rhino.Commands.Result.Success)
                return go.CommandResult();

            double modelTol = doc.ModelAbsoluteTolerance;
            double discretizeTol = modelTol * 0.1;
            double angleTolRad = RhinoMath.ToRadians(0.5);

            // Phase 18.2 — bucket the user's selection.
            var closedCurves = new List<Curve>();
            var openCurves = new List<Curve>();
            var labels = new List<TextEntity>();
            int skippedAnnotations = 0;
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var geom = go.Object(i).Geometry();
                if (geom is Curve cv)
                {
                    if (cv.IsClosed) closedCurves.Add(cv);
                    else openCurves.Add(cv);
                }
                else if (geom is TextEntity te)
                {
                    labels.Add(te);
                }
                else if (geom is AnnotationBase)
                {
                    // Dimensions, leaders, hatches: not supported as labels.
                    skippedAnnotations++;
                }
            }

            if (closedCurves.Count == 0)
            {
                RhinoApp.WriteLine("No closed curves selected — nothing to nest.");
                return Rhino.Commands.Result.Cancel;
            }
            if (skippedAnnotations > 0)
            {
                RhinoApp.WriteLine(
                    $"{skippedAnnotations} non-text annotation(s) skipped (only TextEntity labels supported).");
            }

            // Phase 18.2 — topology partition of closed curves.
            //
            // Build containedBy[i] = list of j's whose region contains curve i,
            // via pairwise Curve.PlanarClosedCurveRelationship. The relationship
            // is symmetric in semantics (AInsideB ↔ BInsideA) but we test each
            // pair exactly once.
            //
            // Common test plane is WorldXY: ReNest input geometry is 2D and
            // (per Phase 18.1) is brought into a local frame downstream anyway.
            // The relationship's `tolerance` parameter is how out-of-plane the
            // curves can be; modelTol*10 is comfortably loose for plate work
            // while staying tight enough to reject curves that genuinely don't
            // share the test plane.
            var topologyPlane = Plane.WorldXY;
            double topologyTol = modelTol * 10.0;
            int n = closedCurves.Count;
            var containedBy = new List<int>[n];
            var bboxArea = new double[n];
            for (int i = 0; i < n; i++)
            {
                containedBy[i] = new List<int>();
                var bb = closedCurves[i].GetBoundingBox(true);
                bboxArea[i] = bb.Diagonal.X * bb.Diagonal.Y;
            }
            for (int i = 0; i < n - 1; i++)
            {
                for (int j = i + 1; j < n; j++)
                {
                    var rel = Curve.PlanarClosedCurveRelationship(
                        closedCurves[i], closedCurves[j], topologyPlane, topologyTol);
                    if (rel == RegionContainment.AInsideB)
                        containedBy[i].Add(j);
                    else if (rel == RegionContainment.BInsideA)
                        containedBy[j].Add(i);
                }
            }

            // Immediate parent = smallest container (smallest bbox area among
            // curves that contain me). Root = walk the parent chain to the
            // top; that's the outer this curve ultimately belongs to.
            //
            // Why root-walk instead of attaching to the immediate parent:
            // for plate cutting, a deeply-nested curve (e.g., etch mark inside
            // a hole inside an outline) needs to travel with the outermost
            // plate. Attaching to the immediate parent (the hole) would only
            // matter if the hole itself were a separately-cut part, but the
            // hole is geometry inside the plate, not its own part. Flattening
            // nested chains to the root is what produces correct cut output.
            var parent = new int[n];
            for (int i = 0; i < n; i++)
            {
                if (containedBy[i].Count == 0) { parent[i] = -1; continue; }
                int smallest = containedBy[i][0];
                for (int k = 1; k < containedBy[i].Count; k++)
                {
                    if (bboxArea[containedBy[i][k]] < bboxArea[smallest])
                        smallest = containedBy[i][k];
                }
                parent[i] = smallest;
            }

            var outers = new List<int>();
            for (int i = 0; i < n; i++)
                if (parent[i] == -1) outers.Add(i);

            // Standalone-inner promotion: any closed curve whose containedBy
            // is empty was already classified as an outer (parent == -1). So
            // the "user selected only the hole, not its outline" case is
            // handled automatically — it shows up as its own outer, which is
            // exactly what the spec calls for. No extra code path needed.

            var innersByOuter = new Dictionary<int, List<int>>();
            foreach (int o in outers) innersByOuter[o] = new List<int>();
            for (int i = 0; i < n; i++)
            {
                if (parent[i] == -1) continue;
                int root = i;
                while (parent[root] != -1) root = parent[root];
                innersByOuter[root].Add(i);
            }

            // Per-outer plane resolution (used for inner-loop mapping AND
            // for the outer's polygon-build / Phase-18.1 frame map). Each
            // outer's TryGetPlane defines the local frame in which its
            // polygon and all its inner loops will live.
            var outerPlanes = new Dictionary<int, Plane>();
            foreach (int o in outers)
            {
                if (!closedCurves[o].TryGetPlane(out Plane p, discretizeTol * 10.0))
                    p = Plane.WorldXY;
                outerPlanes[o] = p;
            }

            // Open-curve association via midpoint Curve.Contains.
            // First-match-wins. Orphan opens (midpoint outside every outer)
            // get logged and dropped — they're typically tracking errors
            // (e.g., a hatch line drawn across two parts).
            var openLoopsByOuter = new Dictionary<int, List<Curve>>();
            foreach (int o in outers) openLoopsByOuter[o] = new List<Curve>();
            int orphanOpenCount = 0;
            foreach (var openCurve in openCurves)
            {
                var midPt = openCurve.PointAtNormalizedLength(0.5);
                int matched = -1;
                foreach (int o in outers)
                {
                    if (closedCurves[o].Contains(midPt, outerPlanes[o], topologyTol) == PointContainment.Inside)
                    {
                        matched = o;
                        break;
                    }
                }
                if (matched >= 0) openLoopsByOuter[matched].Add(openCurve);
                else orphanOpenCount++;
            }
            if (orphanOpenCount > 0)
            {
                RhinoApp.WriteLine(
                    $"{orphanOpenCount} open curve(s) not inside any selected outer — dropped " +
                    "(midpoint outside every outer; likely a stray etch line).");
            }

            // Label association via anchor-point Curve.Contains.
            // PlainText (round-trips through SeaNest's Phase 8/10/11 emit
            // path; font/height/style come from SeaNest defaults — see the
            // Phase 18.2 audit "discard hand-edited label formatting"
            // decision, deferred to Phase 18.3 if it becomes a workflow issue).
            // First match wins; subsequent labels for the same outer warn.
            var labelByOuter = new Dictionary<int, string>();
            int orphanLabelCount = 0;
            int duplicateLabelCount = 0;
            foreach (var label in labels)
            {
                var anchor = label.Plane.Origin;
                int matched = -1;
                foreach (int o in outers)
                {
                    if (closedCurves[o].Contains(anchor, outerPlanes[o], topologyTol) == PointContainment.Inside)
                    {
                        matched = o;
                        break;
                    }
                }
                if (matched < 0) { orphanLabelCount++; continue; }
                if (labelByOuter.ContainsKey(matched))
                {
                    duplicateLabelCount++;
                    continue;
                }
                labelByOuter[matched] = label.PlainText;
            }
            if (orphanLabelCount > 0)
                RhinoApp.WriteLine(
                    $"{orphanLabelCount} label(s) not inside any outer — dropped.");
            if (duplicateLabelCount > 0)
                RhinoApp.WriteLine(
                    $"{duplicateLabelCount} duplicate label(s) — first per outer kept, rest dropped.");

            // Phase 18.2 — parallel lists indexed by outer slot
            // (== eventual PlacementResult.OriginalIndex). The polygon
            // goes to the engine; the outer curve, inner loops (closed +
            // open), and label string all flow through DrawNestingResult
            // which applies the placement transform uniformly.
            var polygons = new List<Polygon>();
            var outerCurvePerPart = new List<Curve>();
            var innerLoopsPerPart = new List<IReadOnlyList<Curve>>();
            var namesPerPart = new List<string>();

            for (int slot = 0; slot < outers.Count; slot++)
            {
                int outerIdx = outers[slot];
                var outerCurve = closedCurves[outerIdx];
                var outerPlane = outerPlanes[outerIdx];

                var converted = CurveToPolygon(outerCurve, outerPlane, discretizeTol, angleTolRad, MaxVertices);
                if (converted == null)
                {
                    RhinoApp.WriteLine($"Part {slot + 1}: outer curve could not convert — skipped.");
                    continue;
                }

                var (poly, mappedOuter, finalTol) = converted.Value;
                if (finalTol > discretizeTol + 1e-12)
                {
                    RhinoApp.WriteLine(
                        $"Part {slot + 1}: simplified at tol {finalTol:G3} to fit {MaxVertices}-vertex cap " +
                        $"(escalated from {discretizeTol:G3}). Visual fidelity preserved via native curve.");
                }

                // Inner loops (closed + open) into the outer's plane-local frame —
                // same Phase 18.1 transform applied so PlacementResult.Transform
                // is coherent for the inner loops too.
                var mappedInners = new List<Curve>();
                foreach (int innerIdx in innersByOuter[outerIdx])
                    mappedInners.Add(MapCurveToLocalFrame(closedCurves[innerIdx], outerPlane));
                foreach (var openLoop in openLoopsByOuter[outerIdx])
                    mappedInners.Add(MapCurveToLocalFrame(openLoop, outerPlane));

                labelByOuter.TryGetValue(outerIdx, out string labelText);

                polygons.Add(poly);
                outerCurvePerPart.Add(mappedOuter);
                innerLoopsPerPart.Add(mappedInners);
                namesPerPart.Add(labelText);
            }

            if (polygons.Count == 0)
            {
                RhinoApp.WriteLine("No curves could be converted. Aborting.");
                return Rhino.Commands.Result.Failure;
            }

            int totalInners = innerLoopsPerPart.Sum(l => l.Count);
            int totalLabels = namesPerPart.Count(s => !string.IsNullOrEmpty(s));
            RhinoApp.WriteLine(
                $"Re-Nest: {polygons.Count} outer part(s), {totalInners} inner loop(s), {totalLabels} label(s).");

            NestRequest request;
            try
            {
                request = new NestRequest(polygons, sheetW, sheetH, sheetT, margin, spacing, rotStep);
            }
            catch (ArgumentException ex)
            {
                RhinoApp.WriteLine($"Invalid parameters: {ex.Message}");
                return Rhino.Commands.Result.Failure;
            }

            var dialog = new ProgressDialog("Re-Nesting... Please Wait");
            dialog.Show();
            Application.Instance.RunIteration();

            NestResponse response;
            try
            {
                var engine = new NestingEngine
                {
                    ProgressCallback = (frac, msg) =>
                    {
                        dialog.UpdateStatus(msg);
                        Application.Instance.RunIteration();
                    }
                };
                response = engine.Nest(request);
            }
            catch (Exception ex)
            {
                dialog.Close();
                RhinoApp.WriteLine($"Re-nesting failed: {ex.Message}");
                return Rhino.Commands.Result.Failure;
            }
            finally
            {
                if (dialog.Visible) dialog.Close();
            }

            var verification = FinalVerifier.Verify(response.Placements);
            if (!verification.IsValid)
            {
                string msg = $"{verification.OverlappingPartCount} parts overlap — report this bug";
                RhinoApp.WriteLine(msg);
                MessageBox.Show(msg, "SeaNest Re-Nest Error", MessageBoxButtons.OK, MessageBoxType.Error);
                return Rhino.Commands.Result.Failure;
            }

            if (response.Placements.Count == 0)
            {
                RhinoApp.WriteLine("No curves could be placed.");
                return Rhino.Commands.Result.Nothing;
            }

            // Phase 18.2 — all three parallel lists populated. innerLoopsPerPart
            // carries the closed-hole and open-etch curves associated with each
            // outer; namesPerPart carries each outer's label string (the existing
            // Phase 8/10/11 emit path computes pole-of-inaccessibility and PCA
            // rotation from the placed polygon, so position and rotation are
            // recomputed at draw time — we only need the string here).
            SeaNestNestCommand.DrawNestingResult(
                doc, response, sheetW, sheetH, sheetT, margin, inToModel, isMetric,
                innerLoopsPerPart: innerLoopsPerPart,
                namesPerPart: namesPerPart,
                outerCurvePerPart: outerCurvePerPart);

            RhinoApp.WriteLine(
                $"SeaNest Re-Nest: placed {response.Placements.Count}/{polygons.Count} parts " +
                $"({totalInners} inner loops, {totalLabels} labels) on {response.SheetCount} sheet(s), " +
                $"utilization {response.Utilization:P1}, {response.ElapsedTime.TotalSeconds:F2}s.");

            if (response.UnplacedIndices.Count > 0)
            {
                var originalNumbers = response.UnplacedIndices.Select(i => (i + 1).ToString());
                RhinoApp.WriteLine($"Unplaced parts: {string.Join(", ", originalNumbers)}");
            }

            return Rhino.Commands.Result.Success;
        }

        /// <summary>
        /// Convert a closed 2D curve to a <see cref="Polygon"/> on the supplied plane.
        /// For a curve that already lies on world XY (typical re-nest input), pass
        /// <see cref="Plane.WorldXY"/>; for tilted/rotated 2D curves the caller's
        /// <c>TryGetPlane</c> result is used so the polygon is built in the curve's
        /// own local frame.
        ///
        /// Phase 18: returns the original Rhino curve alongside the polygon so the
        /// draw path can preserve native NURBS/arc/polyline representation
        /// (mirrors SeaNestNest's outerCurvePerPart). Applies Polygon.SimplifyToTarget
        /// with the same vertex-cap escalation pattern BrepFlattener uses.
        ///
        /// Phase 18.1: the returned curve is mapped into the polygon's source frame
        /// before being handed back. The polygon's UV coords are produced by
        /// <c>plane.ClosestParameter</c>, which is the per-point form of
        /// <c>Transform.PlaneToPlane(plane, Plane.WorldXY)</c>; the native curve has
        /// to live in the same plane-local frame or <see cref="PlacementResult.Transform"/>
        /// (whose step1 = <c>-srcBBox.Min</c> is taken from the polygon's bounding box)
        /// will subtract a plane-local quantity from a world-space curve and place the
        /// result at the wrong world position. For an offset or flipped <c>TryGetPlane</c>
        /// result this produces visible mis-placement; the original SeaNestReNest
        /// regression manifested as a placed curve hundreds of inches off the sheet.
        ///
        /// Symmetric with <c>BrepFlattener.ProjectCurveToPlaneSpace</c> (Phase 15.1) —
        /// same primitive, same purpose: align the native curve with the frame the
        /// polygon was built in. If the caller passed <see cref="Plane.WorldXY"/> for
        /// a curve already at world origin the PlaneToPlane transform is identity.
        ///
        /// Phase 18.2: caller now supplies <paramref name="plane"/> so the same frame
        /// can be reused for any associated inner loops via <see cref="MapCurveToLocalFrame"/>.
        /// </summary>
        private static (Polygon polygon, Curve originalCurve, double finalTolerance)? CurveToPolygon(
            Curve curve, Plane plane, double discretizeTol, double angleTolRad, int maxVertices)
        {
            if (!curve.IsClosed)
                return null;

            Polyline polyline;
            if (!curve.TryGetPolyline(out polyline))
            {
                var polylineCurve = curve.ToPolyline(
                    mainSegmentCount: 0,
                    subSegmentCount: 0,
                    maxAngleRadians: angleTolRad,
                    maxChordLengthRatio: 0,
                    maxAspectRatio: 0,
                    tolerance: discretizeTol,
                    minEdgeLength: 0,
                    maxEdgeLength: 0,
                    keepStartPoint: true);
                if (polylineCurve == null || !polylineCurve.TryGetPolyline(out polyline))
                    return null;
            }

            if (polyline == null || polyline.Count < 3) return null;

            var points = new List<Point2D>(polyline.Count);
            int count = polyline.Count;
            if (polyline.IsClosed && count > 3) count--;
            for (int i = 0; i < count; i++)
            {
                var pt3 = polyline[i];
                if (!plane.ClosestParameter(pt3, out double u, out double v))
                    return null;
                points.Add(new Point2D(u, v));
            }
            if (points.Count < 3) return null;

            Polygon poly;
            try { poly = new Polygon(points); }
            catch (ArgumentException) { return null; }

            Polygon simplified;
            double finalTol;
            try
            {
                simplified = poly.SimplifyToTarget(
                    initialTolerance: discretizeTol,
                    maxVertices: maxVertices,
                    escalationFactor: EscalationFactor,
                    maxEscalations: MaxEscalations,
                    out finalTol);
            }
            catch (ArgumentException)
            {
                simplified = poly;
                finalTol = discretizeTol;
            }

            var localCurve = MapCurveToLocalFrame(curve, plane);
            return (simplified, localCurve, finalTol);
        }

        /// <summary>
        /// Phase 18.1/18.2 — map a Rhino curve into the plane-local frame defined
        /// by <paramref name="plane"/>. This is the matrix form of what
        /// <c>plane.ClosestParameter</c> does per-point: it rigidly rotates and
        /// translates the curve so that <paramref name="plane"/>'s origin maps to
        /// world origin and its X/Y axes map to world X/Y.
        ///
        /// <see cref="Curve.ProjectToPlane"/> is applied first to defend against
        /// curves that drift slightly off the best-fit plane (the rigid PlaneToPlane
        /// transform preserves the perpendicular component, leaving residual Z
        /// otherwise). For a curve already coplanar with <paramref name="plane"/>
        /// this is near-identity. On a null result we keep the un-projected duplicate.
        ///
        /// Used for both the outer curve and every associated inner loop so they
        /// share the same source frame — <see cref="PlacementResult.Transform"/>'s
        /// <c>step1 = -srcBBox.Min</c> is then a coherent quantity for all of them.
        /// </summary>
        private static Curve MapCurveToLocalFrame(Curve curve, Plane plane)
        {
            var local = curve.DuplicateCurve();
            var projected = Curve.ProjectToPlane(local, plane);
            if (projected != null) local = projected;
            var planeToXY = Transform.PlaneToPlane(plane, Plane.WorldXY);
            local.Transform(planeToXY);
            return local;
        }
    }
}