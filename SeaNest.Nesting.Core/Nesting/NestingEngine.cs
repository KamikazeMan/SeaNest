using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Nesting engine — pure algorithm, no Rhino dependencies.
    ///
    /// Dispatches on <see cref="NestRequest.Algorithm"/>:
    ///   - <see cref="NestingAlgorithm.BLF"/>: Phase 1 Bottom-Left-Fill with bbox-corner
    ///     candidates and slide-tightening. Fastest, ~60-75% utilization.
    ///   - <see cref="NestingAlgorithm.NFP_Greedy"/>: NFP-based single-pass placement
    ///     with mirror support, ~78-85% utilization.
    ///   - <see cref="NestingAlgorithm.NFP_Annealed"/>: NFP greedy wrapped in simulated
    ///     annealing, time-budgeted, ~85-92% utilization.
    /// </summary>
    public sealed class NestingEngine
    {
        /// <summary>Fixed step size for slide-tightening, in model units. BLF only.</summary>
        private const double SlideStep = 0.01;

        // Phase 22a — NFP-only proxy simplification.
        // These polygons are used only for NFP collision/placement. Original Rhino
        // curves/native output remain untouched by the command draw path.
        private const int NfpTargetVertices = 80;

        // Start below typical fabrication tolerance but high enough to collapse
        // dense arc/polyline tessellation.
        private const double NfpProxyInitialTolerance = 0.01;

        // Escalate gradually instead of jumping straight to a coarse proxy.
        private const double NfpProxyEscalationFactor = 1.5;

        // Production hard cap. 0.05" is below 1/16" kerf but close enough that
        // we warn when the proxy still cannot reach the target.
        private const double NfpProxyMaxTolerance = 0.05;

        // Safety guards. These reject a proxy that changes the part too much.
        private const double NfpProxyMaxAreaDriftFraction = 0.005; // 0.5%
        private const double NfpProxyMaxBBoxDrift = 0.05;          // inches/model units

        /// <summary>Report progress (0..1) and status text. May be null.</summary>
        public Action<double, string> ProgressCallback { get; set; }

        /// <summary>
        /// Verbose engine-internal diagnostic messages (per-orientation timings, cache
        /// stats, etc). Independent of <see cref="ProgressCallback"/> — diagnostics
        /// don't carry a meaningful progress fraction and shouldn't perturb the
        /// progress bar. Typically wired to a logger or RhinoApp.WriteLine. May be null.
        /// </summary>
        public Action<string> DiagnosticCallback { get; set; }

        /// <summary>
        /// Phase 24b: optional time budget for single-sheet shelf-pack retry
        /// after SA + Phase 23 evacuation. Null = shelf-pack disabled.
        /// NFP_Annealed only; fires when result still has >= 2 sheets.
        /// </summary>
        public TimeSpan? ShelfPackTimeBudget { get; set; }

        public NestResponse Nest(NestRequest request)
        {
            if (request == null) throw new ArgumentNullException(nameof(request));

            // Phase 24a: classify parts before any placement work. Diagnostic
            // only — no behavior change. Output goes to a desktop file to
            // bypass the Rhino F2 500-line cap.
            if (DiagnosticCallback != null)
            {
                PartClassifier.RunDiagnostic(request.Polygons, DiagnosticCallback);
            }

            var stopwatch = Stopwatch.StartNew();

            switch (request.Algorithm)
            {
                case NestingAlgorithm.BLF:
                    return NestBLF(request, stopwatch);

                case NestingAlgorithm.NFP_Greedy:
                    return NestNFP(request, stopwatch, useAnnealing: false);

                case NestingAlgorithm.NFP_Annealed:
                    return NestNFP(request, stopwatch, useAnnealing: true);

                default:
                    throw new InvalidOperationException(
                        $"Unknown nesting algorithm: {request.Algorithm}");
            }
        }

        // ==================================================================
        // NFP path (Phase 2)
        // ==================================================================

        private NestResponse NestNFP(NestRequest request, Stopwatch stopwatch, bool useAnnealing)
        {
            ProgressCallback?.Invoke(0.05, "Building part orientations...");

            // Phase 22a — NFP-only proxy simplification. See NfpTargetVertices
            // and BuildNfpProxy. BLF path is unaffected — it consumes
            // request.Polygons directly.
            var nfpPolygons = SimplifyForNfp(request.Polygons);

            // Step 1: Build all oriented variants. Mirror and rotation baked into each.
            OrientedPart.BuildAll(
                nfpPolygons,
                request.RotationStepDegrees,
                request.AllowMirror,
                out var allOrientations,
                out var orientationsByPart);

            ProgressCallback?.Invoke(0.10, "Initializing NFP cache...");

            // Step 2: Construct lazy NFP cache. Spacing is baked into every entry.
            var cache = new NfpCache(request.Spacing);

            // Step 3: Construct placement engine.
            var engine = new NfpPlacementEngine(request, orientationsByPart, cache);
            engine.DiagnosticLog = DiagnosticCallback;

            // Reset the concave-aware CW filter counters so the end-of-nest
            // summary reports only this run's classifications. Wire the per-
            // anomaly diagnostic sink so the (src-orient, cand-orient) pair
            // of each anomaly surfaces during the run. ComputeLog surfaces
            // expensive (slow or large-input) NFP cache-miss computations to
            // attribute MinkowskiDiff cost — gated by elapsed/vertex
            // thresholds in NoFitPolygon so it doesn't spam on cheap pairs.
            NoFitPolygon.ResetCounters();
            NoFitPolygon.AnomalyLog = DiagnosticCallback;
            NoFitPolygon.ComputeLog = DiagnosticCallback;

            try
            {
                return RunNfpInner(request, stopwatch, useAnnealing, engine, cache, nfpPolygons);
            }
            finally
            {
                NoFitPolygon.AnomalyLog = null;
                NoFitPolygon.ComputeLog = null;
            }
        }

        private NestResponse RunNfpInner(
            NestRequest request,
            Stopwatch stopwatch,
            bool useAnnealing,
            NfpPlacementEngine engine,
            NfpCache cache,
            IReadOnlyList<Polygon> nfpPolygons)
        {
            // Step 4: Place parts.
            NfpPlacementEngine.NestResult result;

            // Largest-bbox-diagonal first — same warm start BLF uses. Both Greedy
            // and Annealed start from this order; Annealed then perturbs around it.
            // Same nfpPolygons used everywhere on the NFP path — bbox is preserved
            // by DP simplification, so warm-start ordering is unaffected.
            var initialOrder = BuildLargestFirstOrder(nfpPolygons);

            if (useAnnealing)
            {
                ProgressCallback?.Invoke(0.15, "Annealing...");

                // SA reports its own progress in the 0..1 range; remap to 0.15..0.95
                // here so the surrounding setup/teardown phases get visible progress too.
                Action<double, string> saProgress = ProgressCallback == null
                    ? (Action<double, string>)null
                    : (frac, status) => ProgressCallback(0.15 + frac * 0.80, status);

                result = SimulatedAnnealing.Optimize(
                    engine,
                    initialOrder,
                    request.TimeBudget,
                    randomSeed: 0,
                    progressCallback: saProgress);
            }
            else
            {
                ProgressCallback?.Invoke(0.15, "Placing parts...");
                result = engine.PlaceAll(initialOrder);
                ProgressCallback?.Invoke(0.95, "Finalizing...");
            }

            // Phase 24b: single-sheet shelf-pack attempt (NFP_Annealed only).
            if (useAnnealing && ShelfPackTimeBudget.HasValue && result.SheetCount >= 2)
            {
                result = TrySingleSheetShelfPack(
                    result, request, nfpPolygons, orientationsByPart, cache);
            }

            // Step 5: Sort placements by original index for stable downstream rendering.
            var placementsSorted = result.Placements
                .OrderBy(p => p.OriginalIndex)
                .ToList();

            double totalPlacedArea = placementsSorted.Sum(p => p.PlacedPolygon.AbsoluteArea);
            double usablePerSheet = request.UsableWidth * request.UsableHeight;

            stopwatch.Stop();
            ProgressCallback?.Invoke(1.0,
                $"Nest complete: {result.SheetCount} sheets, {result.Unplaced.Count} unplaced");

            // Concave-aware CW filter summary. Surfaces only when the filter
            // touched anything during this run. Per-anomaly detail (with
            // src-orient, cand-orient identifiers) was emitted live via
            // AnomalyLog at the moment each anomaly was classified — this
            // line is the run-level rollup. cache.Count is the total number
            // of unique pairwise NFPs computed.
            int slivers = NoFitPolygon.SliverReversalsTotal;
            int holes = NoFitPolygon.HolesPreservedTotal;
            int anomalies = NoFitPolygon.AnomalyReversalsTotal;
            if (slivers > 0 || holes > 0 || anomalies > 0)
            {
                DiagnosticCallback?.Invoke(
                    $"NFP: {cache.Count} pairs computed, " +
                    $"{slivers} slivers reversed, " +
                    $"{holes} holes preserved, " +
                    $"{anomalies} anomalies");
            }

            return new NestResponse(
                placementsSorted,
                new List<int>(result.Unplaced),
                totalPlacedArea,
                usablePerSheet,
                stopwatch.Elapsed);
        }

        // ==================================================================
        // Phase 24b: single-sheet shelf-pack
        // ==================================================================

        private const double ShelfPackHeadroomFraction = 0.70;

        private static readonly string ShelfPackOutputPath =
            Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory),
                "phase24b_shelfpack.txt");

        private enum ShelfSide { Top, Bottom, Left, Right }

        private NfpPlacementEngine.NestResult TrySingleSheetShelfPack(
            NfpPlacementEngine.NestResult originalResult,
            NestRequest request,
            IReadOnlyList<Polygon> nfpPolygons,
            IReadOnlyList<List<OrientedPart>> orientationsByPart,
            NfpCache cache)
        {
            var budget = ShelfPackTimeBudget.Value;
            var sw = Stopwatch.StartNew();
            var log = new List<string>();
            log.Add($"Phase 24b shelf-pack run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            log.Add($"Budget: {budget.TotalSeconds:F0}s, sheets before: {originalResult.SheetCount}");
            log.Add("");

            // Headroom check.
            double totalArea = 0;
            for (int i = 0; i < nfpPolygons.Count; i++)
                totalArea += nfpPolygons[i].AbsoluteArea;
            double sheetArea = request.SheetWidth * request.SheetHeight;
            double headroom = totalArea / sheetArea;

            if (headroom >= ShelfPackHeadroomFraction)
            {
                log.Add($"Headroom check FAILED: total part area {totalArea:F1} / sheet area {sheetArea:F1} = {headroom:P1} >= {ShelfPackHeadroomFraction:P0} limit.");
                log.Add("Shelf-pack skipped — single sheet is mathematically infeasible.");
                WriteShelfPackLog(log);
                DiagnosticCallback?.Invoke("Phase 24b shelf-pack: headroom failed, skipping.");
                return originalResult;
            }

            log.Add($"Headroom check passed: {headroom:P1} < {ShelfPackHeadroomFraction:P0}.");
            log.Add("");

            // Classify all parts.
            var classes = new PartClass[nfpPolygons.Count];
            var majorIndices = new List<int>();
            var minorIndices = new List<int>();
            var irregularIndices = new List<int>();
            var tinyIndices = new List<int>();

            for (int i = 0; i < nfpPolygons.Count; i++)
            {
                classes[i] = PartClassifier.Classify(nfpPolygons[i]);
                switch (classes[i])
                {
                    case PartClass.StripMajor: majorIndices.Add(i); break;
                    case PartClass.StripMinor: minorIndices.Add(i); break;
                    case PartClass.Tiny: tinyIndices.Add(i); break;
                    default: irregularIndices.Add(i); break;
                }
            }

            log.Add($"Classification: {majorIndices.Count} major, {minorIndices.Count} minor, " +
                    $"{irregularIndices.Count} irregular, {tinyIndices.Count} tiny.");
            log.Add("");

            if (majorIndices.Count == 0)
            {
                log.Add("No StripMajor parts — shelf-pack not applicable.");
                WriteShelfPackLog(log);
                DiagnosticCallback?.Invoke("Phase 24b shelf-pack: no StripMajor parts, skipping.");
                return originalResult;
            }

            // Sort StripMajor by longest side descending.
            majorIndices.Sort((a, b) =>
            {
                double la = Math.Max(nfpPolygons[a].BoundingBox.Width, nfpPolygons[a].BoundingBox.Height);
                double lb = Math.Max(nfpPolygons[b].BoundingBox.Width, nfpPolygons[b].BoundingBox.Height);
                return lb.CompareTo(la);
            });

            // Try each strategy.
            var strategies = new[] { ShelfSide.Top, ShelfSide.Bottom, ShelfSide.Left, ShelfSide.Right };
            NfpPlacementEngine.NestResult bestResult = null;

            foreach (var side in strategies)
            {
                if (sw.Elapsed >= budget) break;

                log.Add($"--- Strategy: shelf on {side} ---");

                var attempt = TryShelfStrategy(
                    side, request, nfpPolygons, orientationsByPart, cache,
                    majorIndices, irregularIndices, minorIndices, tinyIndices,
                    log);

                if (attempt != null && attempt.SheetCount == 1 &&
                    attempt.Unplaced.Count == 0)
                {
                    log.Add($"Strategy {side}: SUCCESS — all parts on 1 sheet.");
                    bestResult = attempt;
                    break;
                }

                string reason = attempt == null ? "shelf overflow"
                    : $"{attempt.SheetCount} sheets, {attempt.Unplaced.Count} unplaced";
                log.Add($"Strategy {side}: failed ({reason}).");
                log.Add("");
            }

            log.Add("");
            if (bestResult != null)
            {
                log.Add($"Phase 24b: shelf-pack succeeded ({sw.ElapsedMilliseconds}ms).");
                WriteShelfPackLog(log);
                DiagnosticCallback?.Invoke(
                    $"Phase 24b shelf-pack succeeded — see Desktop\\phase24b_shelfpack.txt " +
                    $"(1 sheet, {sw.ElapsedMilliseconds}ms).");
                return bestResult;
            }

            log.Add($"Phase 24b: all strategies failed, keeping original {originalResult.SheetCount}-sheet layout ({sw.ElapsedMilliseconds}ms).");
            WriteShelfPackLog(log);
            DiagnosticCallback?.Invoke(
                $"Phase 24b shelf-pack failed (all 4 strategies), keeping original layout — " +
                $"see Desktop\\phase24b_shelfpack.txt");
            return originalResult;
        }

        private NfpPlacementEngine.NestResult TryShelfStrategy(
            ShelfSide side,
            NestRequest request,
            IReadOnlyList<Polygon> nfpPolygons,
            IReadOnlyList<List<OrientedPart>> orientationsByPart,
            NfpCache cache,
            List<int> majorIndices,
            List<int> irregularIndices,
            List<int> minorIndices,
            List<int> tinyIndices,
            List<string> log)
        {
            bool horizontal = (side == ShelfSide.Top || side == ShelfSide.Bottom);
            double shelfLength = horizontal ? request.UsableWidth : request.UsableHeight;
            double spacing = request.Spacing;

            // Compute shelf depth and strip positions.
            double shelfDepth = 0;
            var stripPositions = new List<(int partIndex, double x, double y, double rotDeg)>();
            double cursor = request.Margin;

            for (int i = 0; i < majorIndices.Count; i++)
            {
                int idx = majorIndices[i];
                var bbox = nfpPolygons[idx].BoundingBox;
                double longSide = Math.Max(bbox.Width, bbox.Height);
                double shortSide = Math.Min(bbox.Width, bbox.Height);
                bool needsRotate = (horizontal && bbox.Height > bbox.Width) ||
                                   (!horizontal && bbox.Width > bbox.Height);
                double rotDeg = needsRotate ? 90.0 : 0.0;
                double partExtentAlongShelf = horizontal ? longSide : longSide;
                double partExtentPerpendicular = horizontal ? shortSide : shortSide;

                if (cursor + partExtentAlongShelf > shelfLength + request.Margin + 0.01)
                {
                    log.Add($"  StripMajor part {idx} ({longSide:F1}x{shortSide:F1}) overflows shelf at cursor={cursor:F1}, shelfLength={shelfLength:F1}.");
                    return null;
                }

                shelfDepth = Math.Max(shelfDepth, partExtentPerpendicular);

                double px, py;
                switch (side)
                {
                    case ShelfSide.Top:
                        px = cursor;
                        py = request.SheetHeight - request.Margin - partExtentPerpendicular;
                        break;
                    case ShelfSide.Bottom:
                        px = cursor;
                        py = request.Margin;
                        break;
                    case ShelfSide.Left:
                        px = request.Margin;
                        py = cursor;
                        break;
                    default: // Right
                        px = request.SheetWidth - request.Margin - partExtentPerpendicular;
                        py = cursor;
                        break;
                }

                stripPositions.Add((idx, px, py, rotDeg));
                cursor += partExtentAlongShelf + spacing;
            }

            shelfDepth += spacing;
            log.Add($"  Shelf depth: {shelfDepth:F2}, {stripPositions.Count} StripMajor parts placed.");

            // Build reduced-sheet NestRequest for Irregulars.
            double reducedW, reducedH;
            double irregOffsetX = 0, irregOffsetY = 0;

            switch (side)
            {
                case ShelfSide.Top:
                    reducedW = request.SheetWidth;
                    reducedH = request.SheetHeight - shelfDepth;
                    break;
                case ShelfSide.Bottom:
                    reducedW = request.SheetWidth;
                    reducedH = request.SheetHeight - shelfDepth;
                    irregOffsetY = shelfDepth;
                    break;
                case ShelfSide.Left:
                    reducedW = request.SheetWidth - shelfDepth;
                    reducedH = request.SheetHeight;
                    irregOffsetX = shelfDepth;
                    break;
                default: // Right
                    reducedW = request.SheetWidth - shelfDepth;
                    reducedH = request.SheetHeight;
                    break;
            }

            if (reducedW <= 2 * request.Margin || reducedH <= 2 * request.Margin)
            {
                log.Add("  Reduced sheet too small for margin.");
                return null;
            }

            // Place Irregulars on the reduced sheet.
            // Reuse orientationsByPart entries rather than rebuilding.
            var irregPolygons = new List<Polygon>();
            var irregOrientations = new List<List<OrientedPart>>();
            var irregIndexMap = new Dictionary<int, int>();
            for (int i = 0; i < irregularIndices.Count; i++)
            {
                int origIdx = irregularIndices[i];
                irregPolygons.Add(nfpPolygons[origIdx]);
                irregOrientations.Add(orientationsByPart[origIdx]);
                irregIndexMap[i] = origIdx;
            }

            NestRequest reducedRequest;
            try
            {
                reducedRequest = new NestRequest(
                    irregPolygons, reducedW, reducedH,
                    request.SheetThickness, request.Margin, request.Spacing,
                    request.RotationStep, NestingAlgorithm.NFP_Greedy, request.AllowMirror,
                    request.TimeBudget);
            }
            catch
            {
                log.Add("  Reduced NestRequest construction failed.");
                return null;
            }

            var irregEngine = new NfpPlacementEngine(reducedRequest, irregOrientations, cache)
            {
                DiagnosticLog = null,
                MaxCandidateVertices = 512
            };

            var irregOrder = BuildLargestFirstOrder(irregPolygons);
            var irregResult = irregEngine.PlaceAll(irregOrder);

            if (irregResult.SheetCount > 1 || irregResult.Unplaced.Count > 0)
            {
                log.Add($"  Irregular placement: {irregResult.SheetCount} sheets, {irregResult.Unplaced.Count} unplaced — failed.");
                return null;
            }

            log.Add($"  Irregular placement: all {irregPolygons.Count} parts on 1 reduced sheet.");

            // Composite all placements into a single sheet.
            var allPlacements = new List<PlacementResult>();

            // Build OrientedParts for the StripMajor shelf placements so the
            // gap-fill engine can compute NFPs against them. Assign orientation
            // indices past all existing indices to avoid collisions.
            int nextOrientIdx = 0;
            foreach (var perPart in orientationsByPart)
                foreach (var op in perPart)
                    if (op.OrientationIndex >= nextOrientIdx)
                        nextOrientIdx = op.OrientationIndex + 1;

            var preplacedList = new List<NfpPlacementEngine.PreplacedPart>();

            // StripMajor placements (deterministic).
            foreach (var (partIndex, px, py, rotDeg) in stripPositions)
            {
                var sourcePoly = nfpPolygons[partIndex];
                var srcBBox = sourcePoly.BoundingBox;

                double rotRad = rotDeg * Math.PI / 180.0;
                var rotatedNormalized = sourcePoly.MoveToOrigin();
                if (Math.Abs(rotDeg) > 0.01)
                    rotatedNormalized = rotatedNormalized.RotateAround(Point2D.Origin, rotRad);
                var rotBBox = rotatedNormalized.BoundingBox;

                var step1 = Transform2D.Translation(-srcBBox.MinX, -srcBBox.MinY);
                var step2 = Transform2D.RotationDegrees(rotDeg);
                var step3 = Transform2D.Translation(-rotBBox.MinX, -rotBBox.MinY);
                var step4 = Transform2D.Translation(px, py);
                var combined = step1.Then(step2).Then(step3).Then(step4);

                var placedPoly = rotatedNormalized.Translate(px, py);

                var shelfOrientation = OrientedPart.Build(
                    nextOrientIdx++, partIndex, sourcePoly, rotDeg, isMirrored: false);

                var placement = new PlacementResult(
                    originalIndex: partIndex,
                    sheet: 0,
                    transform: combined,
                    rotationDeg: rotDeg,
                    isMirrored: false,
                    sourceBBoxMinX: srcBBox.MinX,
                    sourceBBoxMaxX: srcBBox.MaxX,
                    placedPolygon: placedPoly);

                allPlacements.Add(placement);
                preplacedList.Add(new NfpPlacementEngine.PreplacedPart(
                    partIndex, 0, shelfOrientation, px, py, placement));
            }

            // Irregular placements (offset from reduced sheet to full sheet).
            foreach (var pp in irregResult.Placements)
            {
                int originalIdx = irregIndexMap[pp.OriginalIndex];
                double offsetX = irregOffsetX;
                double offsetY = irregOffsetY;

                var placedPoly = pp.PlacedPolygon.Translate(offsetX, offsetY);
                var offsetTransform = pp.Transform.Then(Transform2D.Translation(offsetX, offsetY));

                var sourcePoly = nfpPolygons[originalIdx];
                var srcBBox = sourcePoly.BoundingBox;

                // Build an OrientedPart for this placed irregular. Use the same
                // rotation/mirror as the engine chose, with a fresh index.
                var irregOrientation = OrientedPart.Build(
                    nextOrientIdx++, originalIdx, sourcePoly,
                    pp.RotationDeg, pp.IsMirrored);

                // Compute the placed position in the full sheet's coordinate space.
                var irregCanonBBox = irregOrientation.CanonicalPolygon.BoundingBox;
                double placedX = placedPoly.BoundingBox.MinX - irregCanonBBox.MinX;
                double placedY = placedPoly.BoundingBox.MinY - irregCanonBBox.MinY;

                var placement = new PlacementResult(
                    originalIndex: originalIdx,
                    sheet: 0,
                    transform: offsetTransform,
                    rotationDeg: pp.RotationDeg,
                    isMirrored: pp.IsMirrored,
                    sourceBBoxMinX: srcBBox.MinX,
                    sourceBBoxMaxX: srcBBox.MaxX,
                    placedPolygon: placedPoly);

                allPlacements.Add(placement);
                preplacedList.Add(new NfpPlacementEngine.PreplacedPart(
                    originalIdx, 0, irregOrientation, placedX, placedY, placement));
            }

            // Gap-fill: place StripMinor + Tiny using PlaceAllWithPreplaced.
            var gapFillIndices = new List<int>();
            gapFillIndices.AddRange(minorIndices);
            gapFillIndices.AddRange(tinyIndices);

            if (gapFillIndices.Count > 0)
            {
                var gapEngine = new NfpPlacementEngine(request, orientationsByPart, cache)
                {
                    DiagnosticLog = null,
                    MaxCandidateVertices = 512
                };

                var gapResult = gapEngine.PlaceAllWithPreplaced(gapFillIndices, preplacedList);

                if (gapResult.SheetCount > 1 || gapResult.Unplaced.Count > 0)
                {
                    log.Add($"  Gap-fill: {gapResult.SheetCount} sheets, {gapResult.Unplaced.Count} unplaced — failed.");
                    return null;
                }

                log.Add($"  Gap-fill: all {gapFillIndices.Count} minor/tiny parts placed.");
                return gapResult;
            }

            // No gap-fill parts needed — return the composite.
            return new NfpPlacementEngine.NestResult(1, allPlacements, new List<int>());
        }

        private void WriteShelfPackLog(List<string> lines)
        {
            try { File.WriteAllLines(ShelfPackOutputPath, lines); }
            catch { /* best effort */ }
        }

        /// <summary>
        /// Largest-bbox-diagonal-first ordering. Same heuristic the BLF engine uses for
        /// its initial pass, and a good warm start for NFP_Greedy.
        /// </summary>
        private static List<int> BuildLargestFirstOrder(IReadOnlyList<Polygon> polygons)
        {
            var indexed = new List<(int idx, double diag)>(polygons.Count);
            for (int i = 0; i < polygons.Count; i++)
                indexed.Add((i, Diagonal(polygons[i].BoundingBox)));

            indexed.Sort((a, b) =>
            {
                int cmp = b.diag.CompareTo(a.diag);
                return cmp != 0 ? cmp : a.idx.CompareTo(b.idx);
            });

            return indexed.Select(t => t.idx).ToList();
        }

        /// <summary>
        /// Apply the NFP-specific simplification cap to every polygon in the input
        /// list. Returns a new list of (possibly) simplified polygons in the same
        /// order; original list is not mutated. Per-part warnings are surfaced via
        /// <see cref="DiagnosticCallback"/> when escalation fires or when even the
        /// max-escalation pass fails to bring a polygon under the NFP cap.
        /// </summary>
        private List<Polygon> SimplifyForNfp(IReadOnlyList<Polygon> polygons)
        {
            var result = new List<Polygon>(polygons.Count);

            for (int i = 0; i < polygons.Count; i++)
            {
                var raw = polygons[i];
                var proxy = BuildNfpProxy(raw, i);
                result.Add(proxy);
            }

            return result;
        }

        private Polygon BuildNfpProxy(Polygon raw, int partIndex)
        {
            if (raw == null)
                throw new ArgumentNullException(nameof(raw));

            if (raw.Count <= 3)
                return raw;

            Polygon bestSafe = raw;
            double bestTol = 0.0;
            double bestAreaDrift = 0.0;
            double bestWidthDrift = 0.0;
            double bestHeightDrift = 0.0;
            bool foundSafe = false;

            double tol = NfpProxyInitialTolerance;

            while (tol <= NfpProxyMaxTolerance + 1e-12)
            {
                Polygon candidate;

                try
                {
                    candidate = raw.SimplifyDouglasPeucker(tol);
                }
                catch (Exception ex)
                {
                    DiagnosticCallback?.Invoke(
                        $"NFP proxy: part {partIndex} simplification failed at tolerance {tol:G4}: {ex.Message}. Using best safe proxy.");
                    break;
                }

                if (!IsAcceptableNfpProxy(
                        raw,
                        candidate,
                        out double areaDrift,
                        out double widthDrift,
                        out double heightDrift,
                        out string rejectReason))
                {
                    DiagnosticCallback?.Invoke(
                        $"NFP proxy: part {partIndex} rejected tolerance {tol:G4}\" — {rejectReason}. " +
                        $"Using {(foundSafe ? "best previous safe proxy" : "raw polygon")}.");

                    break;
                }

                foundSafe = true;
                bestSafe = candidate;
                bestTol = tol;
                bestAreaDrift = areaDrift;
                bestWidthDrift = widthDrift;
                bestHeightDrift = heightDrift;

                if (candidate.Count <= NfpTargetVertices)
                    break;

                tol *= NfpProxyEscalationFactor;
            }

            if (!foundSafe)
                return raw;

            if (bestSafe.Count < raw.Count)
            {
                DiagnosticCallback?.Invoke(
                    $"NFP proxy: part {partIndex} reduced {raw.Count} -> {bestSafe.Count} verts " +
                    $"at tol {bestTol:G4}\"; area drift {bestAreaDrift:P2}, " +
                    $"bbox drift W={bestWidthDrift:G4}\", H={bestHeightDrift:G4}\".");
            }

            if (bestSafe.Count > NfpTargetVertices)
            {
                DiagnosticCallback?.Invoke(
                    $"NFP proxy: part {partIndex} still has {bestSafe.Count} verts after max safe tolerance " +
                    $"{bestTol:G4}\". Target is {NfpTargetVertices}. This part may still dominate NFP runtime.");
            }

            return bestSafe;
        }

        private static bool IsAcceptableNfpProxy(
            Polygon raw,
            Polygon candidate,
            out double areaDriftFraction,
            out double widthDrift,
            out double heightDrift,
            out string rejectReason)
        {
            areaDriftFraction = 0.0;
            widthDrift = 0.0;
            heightDrift = 0.0;
            rejectReason = null;

            if (candidate == null)
            {
                rejectReason = "candidate is null";
                return false;
            }

            if (candidate.Count < 3)
            {
                rejectReason = "candidate has fewer than 3 vertices";
                return false;
            }

            double rawArea = raw.AbsoluteArea;
            double candidateArea = candidate.AbsoluteArea;

            if (rawArea <= 1e-12 || candidateArea <= 1e-12)
            {
                rejectReason = "zero or near-zero area";
                return false;
            }

            areaDriftFraction = Math.Abs(candidateArea - rawArea) / rawArea;

            var rawBox = raw.BoundingBox;
            var candidateBox = candidate.BoundingBox;

            double rawWidth = rawBox.MaxX - rawBox.MinX;
            double rawHeight = rawBox.MaxY - rawBox.MinY;
            double candidateWidth = candidateBox.MaxX - candidateBox.MinX;
            double candidateHeight = candidateBox.MaxY - candidateBox.MinY;

            widthDrift = Math.Abs(candidateWidth - rawWidth);
            heightDrift = Math.Abs(candidateHeight - rawHeight);

            if (areaDriftFraction > NfpProxyMaxAreaDriftFraction)
            {
                rejectReason =
                    $"area drift {areaDriftFraction:P2} exceeds limit {NfpProxyMaxAreaDriftFraction:P2}";
                return false;
            }

            if (widthDrift > NfpProxyMaxBBoxDrift)
            {
                rejectReason =
                    $"bbox width drift {widthDrift:G4}\" exceeds limit {NfpProxyMaxBBoxDrift:G4}\"";
                return false;
            }

            if (heightDrift > NfpProxyMaxBBoxDrift)
            {
                rejectReason =
                    $"bbox height drift {heightDrift:G4}\" exceeds limit {NfpProxyMaxBBoxDrift:G4}\"";
                return false;
            }

            return true;
        }

        // ==================================================================
        // BLF path (Phase 1) — preserved verbatim
        // ==================================================================

        private NestResponse NestBLF(NestRequest request, Stopwatch stopwatch)
        {
            var parts = new List<Part>(request.Polygons.Count);
            for (int i = 0; i < request.Polygons.Count; i++)
            {
                var p = request.Polygons[i];
                parts.Add(new Part(i, p.MoveToOrigin()));
            }

            parts.Sort((a, b) =>
            {
                double da = Diagonal(a.Normalized.BoundingBox);
                double db = Diagonal(b.Normalized.BoundingBox);
                int cmp = db.CompareTo(da);
                return cmp != 0 ? cmp : a.OriginalIndex.CompareTo(b.OriginalIndex);
            });

            double rotStepDeg = request.RotationStepDegrees;
            int rotCount = request.RotationCount;
            var rotationsDeg = new double[rotCount];
            for (int r = 0; r < rotCount; r++)
                rotationsDeg[r] = r * rotStepDeg;

            var sheets = new List<Sheet>();
            var placements = new List<PlacementResult>();
            var unplaced = new List<int>();

            int totalParts = parts.Count;
            int processedParts = 0;

            foreach (var part in parts)
            {
                processedParts++;
                ProgressCallback?.Invoke(
                    (double)processedParts / totalParts,
                    $"Nesting part {processedParts} of {totalParts}...");

                bool placed = false;

                for (int sheetIdx = 0; sheetIdx < sheets.Count && !placed; sheetIdx++)
                {
                    placed = TryPlaceOnSheetBLF(part, sheets[sheetIdx], sheetIdx, request, rotationsDeg, placements);
                }

                if (!placed)
                {
                    var newSheet = new Sheet();
                    sheets.Add(newSheet);
                    int newSheetIdx = sheets.Count - 1;
                    placed = TryPlaceOnSheetBLF(part, newSheet, newSheetIdx, request, rotationsDeg, placements);

                    if (!placed)
                    {
                        sheets.RemoveAt(newSheetIdx);
                        unplaced.Add(part.OriginalIndex);
                    }
                }
            }

            placements.Sort((a, b) => a.OriginalIndex.CompareTo(b.OriginalIndex));

            double totalPlacedArea = placements.Sum(p => p.PlacedPolygon.AbsoluteArea);
            double usablePerSheet = request.UsableWidth * request.UsableHeight;

            stopwatch.Stop();

            return new NestResponse(
                placements,
                unplaced,
                totalPlacedArea,
                usablePerSheet,
                stopwatch.Elapsed);
        }

        private bool TryPlaceOnSheetBLF(
            Part part,
            Sheet sheet,
            int sheetIdx,
            NestRequest request,
            double[] rotationsDeg,
            List<PlacementResult> placements)
        {
            PlacementCandidate best = null;

            double minX = request.Margin;
            double minY = request.Margin;
            double maxX = request.SheetWidth - request.Margin;
            double maxY = request.SheetHeight - request.Margin;

            foreach (double angleDeg in rotationsDeg)
            {
                double angleRad = angleDeg * Math.PI / 180.0;
                var rotated = part.Normalized
                    .RotateAround(Point2D.Origin, angleRad)
                    .MoveToOrigin();

                var rotBBox = rotated.BoundingBox;
                double w = rotBBox.MaxX - rotBBox.MinX;
                double h = rotBBox.MaxY - rotBBox.MinY;

                if (w > request.UsableWidth || h > request.UsableHeight)
                    continue;

                var candidates = GenerateCandidatePositionsBLF(sheet, minX, minY, maxX - w, maxY - h);

                foreach (var (cx, cy) in candidates)
                {
                    var tight = SlideTightenBLF(
                        rotated, cx, cy,
                        minX, minY, maxX - w, maxY - h,
                        sheet.InflatedPlaced);

                    if (tight == null)
                        continue;

                    double tx = tight.Value.x;
                    double ty = tight.Value.y;

                    if (best == null ||
                        ty < best.Y - 1e-9 ||
                        (Math.Abs(ty - best.Y) < 1e-9 && tx < best.X - 1e-9))
                    {
                        best = new PlacementCandidate
                        {
                            X = tx,
                            Y = ty,
                            RotationDeg = angleDeg,
                            RotatedPolygon = rotated
                        };
                    }
                }
            }

            if (best == null) return false;

            var finalPolygon = best.RotatedPolygon.Translate(best.X, best.Y);

            var origBBox = request.Polygons[part.OriginalIndex].BoundingBox;
            var step1 = Transform2D.Translation(-origBBox.MinX, -origBBox.MinY);
            var step2 = Transform2D.RotationDegrees(best.RotationDeg);
            var rotatedOfNormalized = part.Normalized.RotateAround(Point2D.Origin, best.RotationDeg * Math.PI / 180.0);
            var rotBBoxFull = rotatedOfNormalized.BoundingBox;
            var step3 = Transform2D.Translation(-rotBBoxFull.MinX, -rotBBoxFull.MinY);
            var step4 = Transform2D.Translation(best.X, best.Y);
            var combined = step1.Then(step2).Then(step3).Then(step4);

            placements.Add(new PlacementResult(
                part.OriginalIndex,
                sheetIdx,
                combined,
                best.RotationDeg,
                origBBox.MinX,
                origBBox.MaxX,
                finalPolygon));

            sheet.Placed.Add(finalPolygon);
            var inflated = PolygonInflate.Inflate(finalPolygon, request.Spacing);
            foreach (var inf in inflated)
                sheet.InflatedPlaced.Add(inf);

            return true;
        }

        /// <summary>
        /// Generate candidate positions from the INFLATED placed parts' bbox corners.
        /// Using inflated (not raw) placed pieces means candidates naturally sit at the edge
        /// of each part's spacing-buffered zone — placing a candidate there doesn't immediately
        /// overlap the buffer.
        /// </summary>
        private static IEnumerable<(double x, double y)> GenerateCandidatePositionsBLF(
            Sheet sheet,
            double xMin, double yMin, double xMax, double yMax)
        {
            yield return (xMin, yMin);

            foreach (var placed in sheet.InflatedPlaced)
            {
                var bb = placed.BoundingBox;
                yield return (ClampX(bb.MinX, xMin, xMax), ClampY(bb.MaxY, yMin, yMax));
                yield return (ClampX(bb.MaxX, xMin, xMax), ClampY(bb.MinY, yMin, yMax));
                yield return (ClampX(bb.MaxX, xMin, xMax), ClampY(bb.MaxY, yMin, yMax));
                yield return (ClampX(bb.MinX, xMin, xMax), ClampY(bb.MinY, yMin, yMax));
            }
        }

        private static double ClampX(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);
        private static double ClampY(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);

        private static (double x, double y)? SlideTightenBLF(
            Polygon rotated,
            double startX, double startY,
            double xMin, double yMin, double xMax, double yMax,
            List<Polygon> inflatedPlaced)
        {
            double x = startX;
            double y = startY;

            if (!IsValidPositionBLF(rotated, x, y, xMin, yMin, xMax, yMax, inflatedPlaced))
                return null;

            while (y - SlideStep >= yMin &&
                   IsValidPositionBLF(rotated, x, y - SlideStep, xMin, yMin, xMax, yMax, inflatedPlaced))
            {
                y -= SlideStep;
            }

            while (x - SlideStep >= xMin &&
                   IsValidPositionBLF(rotated, x - SlideStep, y, xMin, yMin, xMax, yMax, inflatedPlaced))
            {
                x -= SlideStep;
            }

            while (y - SlideStep >= yMin &&
                   IsValidPositionBLF(rotated, x, y - SlideStep, xMin, yMin, xMax, yMax, inflatedPlaced))
            {
                y -= SlideStep;
            }

            return (x, y);
        }

        private static bool IsValidPositionBLF(
            Polygon rotated,
            double x, double y,
            double xMin, double yMin, double xMax, double yMax,
            List<Polygon> inflatedPlaced)
        {
            if (x < xMin - 1e-9 || y < yMin - 1e-9 || x > xMax + 1e-9 || y > yMax + 1e-9)
                return false;

            var candidate = rotated.Translate(x, y);
            return !OverlapChecker.OverlapsAny(candidate, inflatedPlaced);
        }

        // ==================================================================
        // Shared helpers
        // ==================================================================

        private static double Diagonal(BoundingBox2D bb)
        {
            double w = bb.MaxX - bb.MinX;
            double h = bb.MaxY - bb.MinY;
            return Math.Sqrt(w * w + h * h);
        }

        // ==================================================================
        // BLF-only state types
        // ==================================================================

        private sealed class Part
        {
            public int OriginalIndex { get; }
            public Polygon Normalized { get; }
            public Part(int idx, Polygon normalized) { OriginalIndex = idx; Normalized = normalized; }
        }

        private sealed class Sheet
        {
            public List<Polygon> Placed { get; } = new List<Polygon>();
            public List<Polygon> InflatedPlaced { get; } = new List<Polygon>();
        }

        private sealed class PlacementCandidate
        {
            public double X;
            public double Y;
            public double RotationDeg;
            public Polygon RotatedPolygon;
        }
    }
}