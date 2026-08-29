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

        // Phase 22c — ReNest is a polish/improve pass; default to the smart
        // engine. Nest defaults to BLF for Phase 1 compatibility; ReNest
        // doesn't carry that constraint.
        private const NestingAlgorithm DefaultAlgorithm = NestingAlgorithm.NFP_Annealed;
        private const bool DefaultAllowMirror = true;
        private const double DefaultTimeBudgetSeconds = 30.0;
        private const double DefaultEvacuationBudgetSeconds = 60.0;
        private const double DefaultShelfPackBudgetSeconds = 60.0;
        private const double DefaultBeamRetryBudgetSeconds = 120.0;

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

            // Thickness-aware ReNest: sheet width/height/spacing are prompted
            // PER THICKNESS GROUP inside the group loop below (different
            // thicknesses may come in different sheet sizes). Thickness itself
            // is no longer prompted at all — it is read from each part's
            // LAYER name. Margin, rotation, algorithm, and the tuning budgets
            // stay global (asked once, applied to every group).
            double margin, timeBudgetSeconds;
            double evacuationBudgetSeconds = 0.0;
            double shelfPackBudgetSeconds = 0.0;
            int randomSeed = 0;
            bool interiorSampling = true;
            double beamRetryBudgetSeconds = 0.0;
            RotationStep rotStep;
            NestingAlgorithm algorithm;
            bool allowMirror;

            if (!SeaNestNestCommand.PromptForDouble(doc, "Margin", DefaultMarginIn * inToModel, out margin)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForRotation(out rotStep)) return Rhino.Commands.Result.Cancel;

            // Phase 22c — same algorithm chooser as Nest, but ReNest's default
            // (on Enter) is NFP_Annealed via the overload.
            if (!SeaNestNestCommand.PromptForAlgorithm(DefaultAlgorithm, out algorithm))
                return Rhino.Commands.Result.Cancel;

            // Mirror and TimeBudget are only meaningful for NFP paths. Skip
            // those prompts for BLF (matches Nest's behavior).
            if (algorithm == NestingAlgorithm.BLF)
            {
                allowMirror = false;
                timeBudgetSeconds = DefaultTimeBudgetSeconds;
            }
            else
            {
                if (!SeaNestNestCommand.PromptForBool("Allow part mirroring", DefaultAllowMirror, out allowMirror))
                    return Rhino.Commands.Result.Cancel;

                if (algorithm == NestingAlgorithm.NFP_Annealed)
                {
                    if (!SeaNestNestCommand.PromptForDouble(doc, "Time budget (seconds)", DefaultTimeBudgetSeconds, out timeBudgetSeconds))
                        return Rhino.Commands.Result.Cancel;
                    if (timeBudgetSeconds <= 0)
                    {
                        RhinoApp.WriteLine("Time budget must be positive.");
                        return Rhino.Commands.Result.Failure;
                    }

                    if (!SeaNestNestCommand.PromptForDouble(doc, "Evacuation budget (seconds, 0=off)", DefaultEvacuationBudgetSeconds, out evacuationBudgetSeconds))
                        return Rhino.Commands.Result.Cancel;
                    if (evacuationBudgetSeconds < 0)
                    {
                        RhinoApp.WriteLine("Evacuation budget cannot be negative.");
                        return Rhino.Commands.Result.Failure;
                    }

                    if (!SeaNestNestCommand.PromptForDouble(doc, "Single-sheet shelf-pack budget (seconds, 0=off)", DefaultShelfPackBudgetSeconds, out shelfPackBudgetSeconds))
                        return Rhino.Commands.Result.Cancel;
                    if (shelfPackBudgetSeconds < 0)
                    {
                        RhinoApp.WriteLine("Shelf-pack budget cannot be negative.");
                        return Rhino.Commands.Result.Failure;
                    }

                    var gi = new Rhino.Input.Custom.GetInteger();
                    gi.SetCommandPrompt("Random seed (0=deterministic)");
                    gi.SetDefaultInteger(0);
                    gi.AcceptNothing(true);
                    var giRes = gi.Get();
                    if (giRes == GetResult.Nothing) randomSeed = 0;
                    else if (giRes == GetResult.Number) randomSeed = (int)gi.Number();
                    else return Rhino.Commands.Result.Cancel;

                    if (!SeaNestNestCommand.PromptForBool("Interior sampling fallback", true, out interiorSampling))
                        return Rhino.Commands.Result.Cancel;

                    if (!SeaNestNestCommand.PromptForDouble(doc, "Beam retry budget (seconds, 0=off)", DefaultBeamRetryBudgetSeconds, out beamRetryBudgetSeconds))
                        return Rhino.Commands.Result.Cancel;
                    if (beamRetryBudgetSeconds < 0)
                    {
                        RhinoApp.WriteLine("Beam retry budget cannot be negative.");
                        return Rhino.Commands.Result.Failure;
                    }
                }
                else
                {
                    timeBudgetSeconds = DefaultTimeBudgetSeconds;
                }
            }

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

            // Phase 18.2 — bucket the user's selection. Thickness-aware: for
            // closed curves the LAYER NAME is captured alongside the geometry
            // so parts can be classified by thickness. Open curves and text
            // stay layer-agnostic — they are never nested, only associated to
            // a containing part as ride-along etch / labels.
            var closedCurvesWithLayer = new List<(Curve Curve, string Layer)>();
            var openCurves = new List<Curve>();
            var labels = new List<TextEntity>();
            int skippedAnnotations = 0;
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var objRef = go.Object(i);
                var geom = objRef.Geometry();
                if (geom is Curve cv)
                {
                    if (cv.IsClosed)
                    {
                        string layerName = "";
                        var rhObj = objRef.Object();
                        if (rhObj != null)
                        {
                            int li = rhObj.Attributes.LayerIndex;
                            if (li >= 0 && li < doc.Layers.Count)
                                layerName = doc.Layers[li].Name ?? "";
                        }
                        closedCurvesWithLayer.Add((cv, layerName));
                    }
                    else
                    {
                        openCurves.Add(cv);
                    }
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

            if (closedCurvesWithLayer.Count == 0)
            {
                RhinoApp.WriteLine("No closed curves selected — nothing to nest.");
                return Rhino.Commands.Result.Cancel;
            }
            if (skippedAnnotations > 0)
            {
                RhinoApp.WriteLine(
                    $"{skippedAnnotations} non-text annotation(s) skipped (only TextEntity labels supported).");
            }

            // ----------------------------------------------------------------
            // Thickness classification. Three layer-name formats parse to
            // decimal inches (TryParseThicknessLayer): fraction with slash or
            // underscore ("3/16", "3_16" -> 0.1875), decimal inches ("0.375"),
            // millimeters ("4mm" -> 4/25.4). Excluded layers (PLATE_OUTLINE,
            // TEXT, SCRIBE_LINES, Default) are sheet boundaries / marks, never
            // cut parts — ignored with a count. Anything else is unrecognized:
            // skipped and reported, never nested, never a crash.
            // ----------------------------------------------------------------
            var thicknessCurves = new List<(Curve Curve, double Inches, string Notation)>();
            var excludedCounts = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);
            var unrecognizedCounts = new Dictionary<string, int>(StringComparer.OrdinalIgnoreCase);

            foreach (var (cv, layerName) in closedCurvesWithLayer)
            {
                if (IsExcludedLayer(layerName))
                {
                    excludedCounts.TryGetValue(layerName, out int ec);
                    excludedCounts[layerName] = ec + 1;
                }
                else if (TryParseThicknessLayer(layerName, out double inches))
                {
                    thicknessCurves.Add((cv, inches, layerName.Trim()));
                }
                else
                {
                    unrecognizedCounts.TryGetValue(layerName, out int uc);
                    unrecognizedCounts[layerName] = uc + 1;
                }
            }

            foreach (var kv in excludedCounts)
                RhinoApp.WriteLine($"{kv.Value} curve(s) on excluded layer '{kv.Key}' ignored (not cut parts).");
            foreach (var kv in unrecognizedCounts)
                RhinoApp.WriteLine($"{kv.Value} curve(s) on unrecognized layer '{kv.Key}' skipped.");

            if (thicknessCurves.Count == 0)
            {
                RhinoApp.WriteLine(
                    "No closed curves on recognized thickness layers (e.g. 3_16, 1/4, 0.375, 4mm) — nothing to nest.");
                return Rhino.Commands.Result.Cancel;
            }

            // ----------------------------------------------------------------
            // Group by thickness VALUE with 0.001" tolerance: "3/16" and
            // "0.1875" are the same material and merge; genuinely different
            // thicknesses stay separate — 4mm (0.15748") vs 3/16" (0.1875")
            // differ by ~0.03" and NEVER merge. Groups ascend by thickness.
            // Comparison is against the group's first-seen value (no drift
            // chaining; real plate thicknesses are far apart).
            // ----------------------------------------------------------------
            const double ThicknessMergeTolerance = 0.001;
            var sortedByThickness = thicknessCurves.OrderBy(t => t.Inches).ToList();
            var groups = new List<(double Inches, string Notation, List<Curve> Curves)>();
            foreach (var tc in sortedByThickness)
            {
                if (groups.Count > 0 &&
                    Math.Abs(tc.Inches - groups[groups.Count - 1].Inches) <= ThicknessMergeTolerance)
                {
                    groups[groups.Count - 1].Curves.Add(tc.Curve);
                }
                else
                {
                    groups.Add((tc.Inches, tc.Notation, new List<Curve> { tc.Curve }));
                }
            }

            RhinoApp.WriteLine(
                $"Thickness groups: {groups.Count} — " +
                string.Join(", ", groups.Select(g => $"{g.Notation} ({g.Curves.Count} curve(s))")));

            // ================================================================
            // Per-thickness-group nesting. Each group runs the full existing
            // pipeline (topology partition → association → convert → engine →
            // verify → draw) on ITS OWN curves with ITS OWN sheet size and
            // spacing, stacked below the previous group's output.
            //
            // Holes sit on the SAME thickness layer as their parent's outer
            // profile (per shop convention), so running the topology partition
            // per group attaches every hole to a parent in the same group —
            // the existing attachment logic works unchanged.
            //
            // Open curves and labels live in shared pools; each group consumes
            // the ones contained in its parts, and leftovers are reported once
            // after all groups (an open curve inside a 3/8" part must not be
            // reported as an orphan while the 1/4" group is nesting).
            // ================================================================
            double groupBaseYOffset = 0.0;
            double groupGapModel = 12.0 * inToModel;
            int totalSheetsAll = 0, totalPlacedAll = 0, totalPartsAll = 0;
            bool anyGroupFailed = false;
            var groupSummaries = new List<string>();

            // Outcome codes: 0 = ok, 1 = nothing placed, 2 = verifier failed,
            // 3 = conversion/engine failure. The loop reports and continues on
            // 1/2/3 so one bad group doesn't abandon the others.
            int NestGroup(
                List<Curve> closedCurves,
                double thicknessInches, string thicknessNotation,
                double sheetW, double sheetH, double spacing,
                double groupBaseY,
                out double consumedHeight, out string groupSummary)
            {
            consumedHeight = 0.0;
            groupSummary = null;

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
            var consumedOpens = new List<Curve>();
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
                if (matched >= 0)
                {
                    openLoopsByOuter[matched].Add(openCurve);
                    consumedOpens.Add(openCurve);
                }
                // Unmatched opens stay in the shared pool: they may belong to a
                // part in a LATER thickness group. Leftovers after all groups
                // are reported once by the outer loop.
            }
            foreach (var c in consumedOpens) openCurves.Remove(c);

            // Label association via anchor-point Curve.Contains.
            // PlainText (round-trips through SeaNest's Phase 8/10/11 emit
            // path; font/height/style come from SeaNest defaults — see the
            // Phase 18.2 audit "discard hand-edited label formatting"
            // decision, deferred to Phase 18.3 if it becomes a workflow issue).
            // First match wins; subsequent labels for the same outer warn.
            var labelByOuter = new Dictionary<int, string>();
            int duplicateLabelCount = 0;
            var consumedLabels = new List<TextEntity>();
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
                // Unmatched labels stay in the shared pool for later groups;
                // leftovers are reported once by the outer loop.
                if (matched < 0) continue;
                consumedLabels.Add(label);
                if (labelByOuter.ContainsKey(matched))
                {
                    duplicateLabelCount++;
                    continue;
                }
                labelByOuter[matched] = label.PlainText;
            }
            foreach (var l in consumedLabels) labels.Remove(l);
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
                RhinoApp.WriteLine($"[{thicknessNotation}] No curves could be converted — group skipped.");
                return 3;
            }

            int totalInners = innerLoopsPerPart.Sum(l => l.Count);
            int totalLabels = namesPerPart.Count(s => !string.IsNullOrEmpty(s));
            RhinoApp.WriteLine(
                $"[{thicknessNotation}] {polygons.Count} outer part(s), {totalInners} inner loop(s), {totalLabels} label(s).");

            NestRequest request;
            try
            {
                request = new NestRequest(
                    polygons, sheetW, sheetH, thicknessInches, margin, spacing, rotStep,
                    algorithm,
                    allowMirror,
                    TimeSpan.FromSeconds(timeBudgetSeconds));
            }
            catch (ArgumentException ex)
            {
                RhinoApp.WriteLine($"[{thicknessNotation}] Invalid parameters: {ex.Message} — group skipped.");
                return 3;
            }

            var dialog = new ProgressDialog($"Re-Nesting {thicknessNotation}... Please Wait");
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
                    },
                    DiagnosticCallback = msg => RhinoApp.WriteLine(msg),
                    RandomSeed = randomSeed,
                    EvacuationTimeBudget = evacuationBudgetSeconds > 0
                        ? TimeSpan.FromSeconds(evacuationBudgetSeconds)
                        : (TimeSpan?)null,
                    ShelfPackTimeBudget = shelfPackBudgetSeconds > 0
                        ? TimeSpan.FromSeconds(shelfPackBudgetSeconds)
                        : (TimeSpan?)null,
                    EnableInteriorSampling = interiorSampling,
                    BeamRetryTimeBudget = beamRetryBudgetSeconds > 0
                        ? TimeSpan.FromSeconds(beamRetryBudgetSeconds)
                        : (TimeSpan?)null
                };
                response = engine.Nest(request);
            }
            catch (Exception ex)
            {
                dialog.Close();
                RhinoApp.WriteLine($"[{thicknessNotation}] Re-nesting failed: {ex.Message} — group skipped.");
                return 3;
            }
            finally
            {
                if (dialog.Visible) dialog.Close();
            }

            var verification = FinalVerifier.Verify(response.Placements);
            if (!verification.IsValid)
            {
                string msg = $"[{thicknessNotation}] {verification.OverlappingPartCount} parts overlap — report this bug";
                RhinoApp.WriteLine(msg);
                MessageBox.Show(msg, "SeaNest Re-Nest Error", MessageBoxButtons.OK, MessageBoxType.Error);
                return 2;
            }

            if (response.Placements.Count == 0)
            {
                RhinoApp.WriteLine($"[{thicknessNotation}] No curves could be placed.");
                return 1;
            }

            // Phase 18.2 — all three parallel lists populated. innerLoopsPerPart
            // carries the closed-hole and open-etch curves associated with each
            // outer; namesPerPart carries each outer's label string (the existing
            // Phase 8/10/11 emit path computes pole-of-inaccessibility and PCA
            // rotation from the placed polygon, so position and rotation are
            // recomputed at draw time — we only need the string here).
            consumedHeight = SeaNestNestCommand.DrawNestingResult(
                doc, response, sheetW, sheetH, thicknessInches, margin, inToModel, isMetric,
                innerLoopsPerPart: innerLoopsPerPart,
                namesPerPart: namesPerPart,
                outerCurvePerPart: outerCurvePerPart,
                baseYOffset: groupBaseY,
                sheetThicknessLabel: thicknessNotation);

            groupSummary =
                $"{thicknessNotation} ({thicknessInches:0.####}\"): " +
                $"{response.Placements.Count}/{polygons.Count} part(s), " +
                $"{response.SheetCount} sheet(s), utilization {response.Utilization:P1}, " +
                $"{response.ElapsedTime.TotalSeconds:F2}s";
            RhinoApp.WriteLine($"  {groupSummary}");

            if (response.UnplacedIndices.Count > 0)
            {
                var originalNumbers = response.UnplacedIndices.Select(i => (i + 1).ToString());
                RhinoApp.WriteLine($"  [{thicknessNotation}] Unplaced parts: {string.Join(", ", originalNumbers)}");
            }

            totalSheetsAll += response.SheetCount;
            totalPlacedAll += response.Placements.Count;
            totalPartsAll += polygons.Count;
            return 0;
            } // end NestGroup

            // ================================================================
            // Drive the groups, ascending thickness. Sheet width/height and
            // spacing are prompted PER GROUP (different thicknesses may come
            // in different sheet sizes); a Cancel on any prompt aborts the
            // command (groups already drawn stay in the document).
            // ================================================================
            int groupNum = 0;
            foreach (var grp in groups)
            {
                groupNum++;
                RhinoApp.WriteLine(
                    $"Nesting {grp.Notation} parts ({grp.Inches:0.####}\") — " +
                    $"{grp.Curves.Count} closed curve(s), group {groupNum} of {groups.Count}:");

                if (!SeaNestNestCommand.PromptForDouble(doc, $"Sheet width for {grp.Notation}", DefaultSheetWidthIn * inToModel, out double grpSheetW))
                    return Rhino.Commands.Result.Cancel;
                if (!SeaNestNestCommand.PromptForDouble(doc, $"Sheet height for {grp.Notation}", DefaultSheetHeightIn * inToModel, out double grpSheetH))
                    return Rhino.Commands.Result.Cancel;
                if (!SeaNestNestCommand.PromptForDouble(doc, $"Spacing for {grp.Notation}", DefaultSpacingIn * inToModel, out double grpSpacing))
                    return Rhino.Commands.Result.Cancel;

                int outcome = NestGroup(
                    grp.Curves, grp.Inches, grp.Notation,
                    grpSheetW, grpSheetH, grpSpacing,
                    groupBaseYOffset,
                    out double consumed, out string summary);

                if (summary != null) groupSummaries.Add(summary);
                if (outcome != 0)
                {
                    anyGroupFailed = true;
                    continue; // next thickness group; nothing drawn for this one
                }

                groupBaseYOffset += consumed + groupGapModel;
            }

            // Leftover opens/labels: contained in no part of any group.
            if (openCurves.Count > 0)
                RhinoApp.WriteLine(
                    $"{openCurves.Count} open curve(s) not inside any nested part — dropped.");
            if (labels.Count > 0)
                RhinoApp.WriteLine(
                    $"{labels.Count} label(s) not inside any nested part — dropped.");

            RhinoApp.WriteLine(
                $"SeaNest Re-Nest (thickness-aware): {groups.Count} group(s), " +
                $"{totalPlacedAll}/{totalPartsAll} part(s) placed on {totalSheetsAll} sheet(s) total.");
            foreach (var s in groupSummaries)
                RhinoApp.WriteLine($"  {s}");

            return anyGroupFailed ? Rhino.Commands.Result.Failure : Rhino.Commands.Result.Success;
        }

        // ------------------------------------------------------------------
        // Thickness-layer parsing
        // ------------------------------------------------------------------

        private static readonly string[] ExcludedLayers =
            { "PLATE_OUTLINE", "TEXT", "SCRIBE_LINES", "Default" };

        private static bool IsExcludedLayer(string layerName)
        {
            if (string.IsNullOrWhiteSpace(layerName)) return true;
            string trimmed = layerName.Trim();
            foreach (var ex in ExcludedLayers)
                if (string.Equals(trimmed, ex, StringComparison.OrdinalIgnoreCase))
                    return true;
            return false;
        }

        /// <summary>
        /// Parse a layer name as a plate thickness in decimal inches.
        /// Three formats:
        ///   Fraction, slash or underscore: "3/16", "3_16" -> 0.1875;
        ///     "1/4", "1_4" -> 0.25.
        ///   Decimal inches: "0.1875", "0.375" -> as-is.
        ///   Millimeters: "4mm", "6.5mm" (case-insensitive, optional space)
        ///     -> value / 25.4.
        /// Anything else is not a thickness layer. InvariantCulture decimal
        /// parsing so a comma-decimal OS locale can't change grouping.
        /// </summary>
        private static bool TryParseThicknessLayer(string layerName, out double inches)
        {
            inches = 0.0;
            if (string.IsNullOrWhiteSpace(layerName)) return false;
            string s = layerName.Trim();

            // Millimeters: "<number>mm"
            if (s.EndsWith("mm", StringComparison.OrdinalIgnoreCase))
            {
                string num = s.Substring(0, s.Length - 2).Trim();
                if (double.TryParse(num, System.Globalization.NumberStyles.Float,
                        System.Globalization.CultureInfo.InvariantCulture, out double mm)
                    && mm > 0)
                {
                    inches = mm / 25.4;
                    return true;
                }
                return false;
            }

            // Fraction: "N/M" or "N_M"
            int sep = s.IndexOfAny(new[] { '/', '_' });
            if (sep > 0 && sep < s.Length - 1)
            {
                string numPart = s.Substring(0, sep);
                string denPart = s.Substring(sep + 1);
                if (int.TryParse(numPart, out int numerator)
                    && int.TryParse(denPart, out int denominator)
                    && numerator > 0 && denominator > 0)
                {
                    inches = (double)numerator / denominator;
                    return true;
                }
                return false;
            }

            // Decimal inches: "0.375", ".375", "0.25"
            if (double.TryParse(s, System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture, out double dec)
                && dec > 0)
            {
                inches = dec;
                return true;
            }

            return false;
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