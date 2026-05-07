using System;
using System.Collections.Generic;
using System.Diagnostics;
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

        /// <summary>Report progress (0..1) and status text. May be null.</summary>
        public Action<double, string> ProgressCallback { get; set; }

        /// <summary>
        /// Verbose engine-internal diagnostic messages (per-orientation timings, cache
        /// stats, etc). Independent of <see cref="ProgressCallback"/> — diagnostics
        /// don't carry a meaningful progress fraction and shouldn't perturb the
        /// progress bar. Typically wired to a logger or RhinoApp.WriteLine. May be null.
        /// </summary>
        public Action<string> DiagnosticCallback { get; set; }

        public NestResponse Nest(NestRequest request)
        {
            if (request == null) throw new ArgumentNullException(nameof(request));

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

            // Step 1: Build all oriented variants. Mirror and rotation baked into each.
            OrientedPart.BuildAll(
                request.Polygons,
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
            // anomaly diagnostic sink so the (src-orient, cand-orient) pair of
            // each anomaly surfaces during the run, not just in a count.
            NoFitPolygon.ResetCounters();
            NoFitPolygon.AnomalyLog = DiagnosticCallback;

            try
            {
                return RunNfpInner(request, stopwatch, useAnnealing, engine, cache);
            }
            finally
            {
                NoFitPolygon.AnomalyLog = null;
            }
        }

        private NestResponse RunNfpInner(
            NestRequest request,
            Stopwatch stopwatch,
            bool useAnnealing,
            NfpPlacementEngine engine,
            NfpCache cache)
        {
            // Step 4: Place parts.
            NfpPlacementEngine.NestResult result;

            // Largest-bbox-diagonal first — same warm start BLF uses. Both Greedy
            // and Annealed start from this order; Annealed then perturbs around it.
            var initialOrder = BuildLargestFirstOrder(request.Polygons);

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