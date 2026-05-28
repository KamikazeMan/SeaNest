using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// NFP-based placement engine.
    ///
    /// Phase Smart-Placement upgrade:
    /// Instead of taking only the single bottom-left vertex from the feasible
    /// region, this engine now collects multiple candidate vertices from the
    /// feasible region and scores them for compactness, used sheet area, used
    /// sheet height, used sheet length, and contact with sheet/part edges.
    ///
    /// This keeps the existing NFP/IFP/Clipper architecture but makes the local
    /// placement decision smarter.
    /// </summary>
    public sealed class NfpPlacementEngine
    {
        private readonly NestRequest _request;
        private readonly IReadOnlyList<List<OrientedPart>> _orientationsByPart;
        private readonly NfpCache _nfpCache;

        public Action<string> DiagnosticLog { get; set; }

        public bool EnableInteriorSampling { get; set; } = false;
        public double? InteriorSamplingStep { get; set; } = null;

        public TimeSpan? BeamRetryTimeBudget { get; set; }
        public int BeamWidth { get; set; } = 32;
        public int PlacementsPerPart { get; set; } = 4;
        public int BeamMaxCandidates { get; set; } = 256;

        public IReadOnlyList<int> CriticalPartIndices { get; set; }
        public HashSet<int> IrregularPartIndices { get; set; }

        private static readonly string InteriorFallbackLogPath =
            Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory),
                "phase26_interior.txt");

        private static readonly List<string> InteriorFallbackLog = new List<string>();

        public NfpPlacementEngine(
            NestRequest request,
            IReadOnlyList<List<OrientedPart>> orientationsByPart,
            NfpCache nfpCache)
        {
            _request = request ?? throw new ArgumentNullException(nameof(request));
            _orientationsByPart = orientationsByPart ?? throw new ArgumentNullException(nameof(orientationsByPart));
            _nfpCache = nfpCache ?? throw new ArgumentNullException(nameof(nfpCache));
        }

        public NestResult PlaceAll(IReadOnlyList<int> partOrder)
        {
            if (partOrder == null) throw new ArgumentNullException(nameof(partOrder));

            InteriorFallbackLog.Clear();

            var sheets = new List<SheetState>();
            var placements = new List<PlacementResult>();
            var unplaced = new List<int>();

            foreach (int partIndex in partOrder)
            {
                var orientations = _orientationsByPart[partIndex];

                bool placed = false;

                for (int sheetIdx = 0; sheetIdx < sheets.Count && !placed; sheetIdx++)
                {
                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        sheets[sheetIdx],
                        sheetIdx,
                        placements);
                }

                if (!placed)
                {
                    var newSheet = new SheetState();
                    sheets.Add(newSheet);

                    int newSheetIdx = sheets.Count - 1;

                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        newSheet,
                        newSheetIdx,
                        placements);

                    if (!placed)
                    {
                        sheets.RemoveAt(newSheetIdx);
                        unplaced.Add(partIndex);
                    }
                }
            }

            FlushInteriorFallbackLog();

            return new NestResult(sheets.Count, placements, unplaced);
        }

        public readonly struct PreplacedPart
        {
            public int OriginalIndex { get; }
            public int Sheet { get; }
            public OrientedPart Orientation { get; }
            public double X { get; }
            public double Y { get; }
            public PlacementResult Placement { get; }

            public PreplacedPart(
                int originalIndex, int sheet, OrientedPart orientation,
                double x, double y, PlacementResult placement)
            {
                OriginalIndex = originalIndex;
                Sheet = sheet;
                Orientation = orientation;
                X = x;
                Y = y;
                Placement = placement;
            }
        }

        public NestResult PlaceAllWithPreplaced(
            IReadOnlyList<int> partOrder,
            IReadOnlyList<PreplacedPart> preplaced)
        {
            if (partOrder == null) throw new ArgumentNullException(nameof(partOrder));
            if (preplaced == null) throw new ArgumentNullException(nameof(preplaced));

            var sheets = new List<SheetState>();
            var placements = new List<PlacementResult>();
            var unplaced = new List<int>();

            // Seed sheet state from pre-placed parts.
            foreach (var pp in preplaced)
            {
                while (sheets.Count <= pp.Sheet)
                    sheets.Add(new SheetState());

                sheets[pp.Sheet].Placed.Add(
                    new PlacedItem(pp.Orientation, pp.X, pp.Y));

                placements.Add(pp.Placement);
            }

            // Clear forbidden-region caches on all seeded sheets so the first
            // real placement rebuilds them against the full pre-placed state.
            foreach (var sheet in sheets)
                sheet.ForbiddenByOrientation.Clear();

            // Now place the new parts normally.
            foreach (int partIndex in partOrder)
            {
                var orientations = _orientationsByPart[partIndex];

                bool placed = false;

                for (int sheetIdx = 0; sheetIdx < sheets.Count && !placed; sheetIdx++)
                {
                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        sheets[sheetIdx],
                        sheetIdx,
                        placements);
                }

                if (!placed)
                {
                    var newSheet = new SheetState();
                    sheets.Add(newSheet);

                    int newSheetIdx = sheets.Count - 1;

                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        newSheet,
                        newSheetIdx,
                        placements);

                    if (!placed)
                    {
                        sheets.RemoveAt(newSheetIdx);
                        unplaced.Add(partIndex);
                    }
                }
            }

            return new NestResult(sheets.Count, placements, unplaced);
        }

        public sealed class NestResult
        {
            public int SheetCount { get; }
            public IReadOnlyList<PlacementResult> Placements { get; }
            public IReadOnlyList<int> Unplaced { get; }

            public NestResult(
                int sheetCount,
                IReadOnlyList<PlacementResult> placements,
                IReadOnlyList<int> unplaced)
            {
                SheetCount = sheetCount;
                Placements = placements;
                Unplaced = unplaced;
            }
        }

        // ------------------------------------------------------------------
        // Placement constants
        // ------------------------------------------------------------------

        private const int TryPlaceOnSheetTimeBudgetSeconds = 30;

        private const double BLVertexSanityFactor = 10.0;

        private const double OverlapTolerance = 1e-4;

        private const int DefaultMaxCandidateVertices = 96;

        public int MaxCandidateVertices { get; set; } = DefaultMaxCandidateVertices;

        private const int MaxInteriorCandidatesPerOrientation = 512;

        // Phase 22b — exact-demand parallel NFP gathering.
        // Below this count, Parallel.For overhead can cost more than it saves.
        private const int ParallelNfpPlacedThreshold = 3;

        // Cap worker count so Rhino remains responsive and Clipper/GC pressure stays sane.
        private const int MaxParallelNfpWorkers = 8;

        // Phase 28.1.5 (TEMPORARY DIAGNOSTIC): when true, the forbidden-region
        // build always runs serially, bypassing BuildForbiddenRegionParallel.
        // Used to localize beam-search nondeterminism: run the benchmark twice
        // with this true (serial) and twice with it false (parallel), diff the
        // logs. If serial is deterministic and parallel isn't, the parallel
        // path is the source despite the clean Lazy/Clipper code review.
        //
        // Default is TRUE for the current diagnostic run. To test the parallel
        // path, set the SEANEST_PARALLEL_FORBIDDEN environment variable to "1"
        // (any non-null/non-empty value flips this to false → parallel).
        // Remove this toggle entirely once the source is localized.
        private static readonly bool ForceSerialForbiddenRegion =
            string.IsNullOrEmpty(
                Environment.GetEnvironmentVariable("SEANEST_PARALLEL_FORBIDDEN"));

        private const double CandidateDuplicateTolerance = 0.001;

        private const double ContactTolerance = 0.02;

        // Score weights. Lower total score is better.
        private const double UsedAreaWeight = 100.0;
        private const double UsedTopWeight = 25.0;
        private const double UsedRightWeight = 5.0;
        private const double BottomBiasWeight = 0.50;
        private const double LeftBiasWeight = 0.10;
        private const double ContactRewardWeight = 30.0;
        private const double LooseIslandPenaltyWeight = 4.0;

        // ------------------------------------------------------------------
        // Per-sheet placement
        // ------------------------------------------------------------------

        private bool TryPlaceOnSheet(
            int partIndex,
            List<OrientedPart> orientations,
            SheetState sheet,
            int sheetIdx,
            List<PlacementResult> placements)
        {
            BestPlacement best = null;
            int overlapRejections = 0;

            long tForbidden = 0;
            long tFeasible = 0;
            long tCandidateSweep = 0;

            int orientationsTried = 0;
            int orientationsFit = 0;
            int candidateCount = 0;
            int validCandidateCount = 0;

            var sw = new System.Diagnostics.Stopwatch();

            var attemptStopwatch = System.Diagnostics.Stopwatch.StartNew();
            var timeBudget = TimeSpan.FromSeconds(TryPlaceOnSheetTimeBudgetSeconds);

            foreach (var orientation in orientations)
            {
                if (attemptStopwatch.Elapsed > timeBudget)
                {
                    if (best != null)
                    {
                        DiagnosticLog?.Invoke(
                            $"  Part {partIndex} budget exceeded after " +
                            $"{attemptStopwatch.Elapsed.TotalSeconds:F1}s on sheet {sheetIdx} " +
                            $"— using best-so-far " +
                            $"({orientationsTried}/{orientations.Count} orientations checked, " +
                            $"{orientationsFit} fit, {validCandidateCount}/{candidateCount} candidates valid).");
                    }
                    else
                    {
                        DiagnosticLog?.Invoke(
                            $"  Part {partIndex} budget exceeded after " +
                            $"{attemptStopwatch.Elapsed.TotalSeconds:F1}s on sheet {sheetIdx} " +
                            $"— no placement found in time, falling through to next sheet " +
                            $"({orientationsTried}/{orientations.Count} orientations checked, " +
                            $"{orientationsFit} fit, {validCandidateCount}/{candidateCount} candidates valid).");
                    }

                    break;
                }

                orientationsTried++;

                var ifp = InnerFitPolygon.Compute(
                    orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                if (!ifp.HasValue)
                    continue;

                orientationsFit++;

                var ifpBox = ifp.Value;

                sw.Restart();
                Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                sw.Stop();
                tForbidden += sw.ElapsedMilliseconds;

                sw.Restart();
                Paths64 feasible = ComputeFeasibleRegion(ifpBox, forbidden);
                sw.Stop();
                tFeasible += sw.ElapsedMilliseconds;

                if (feasible.Count == 0)
                    continue;

                sw.Restart();
                var candidates = FindCandidateVertices(
                    feasible,
                    MaxCandidateVertices);
                sw.Stop();
                tCandidateSweep += sw.ElapsedMilliseconds;

                if (candidates.Count == 0)
                    continue;

                candidateCount += candidates.Count;

                foreach (var candidate in candidates)
                {
                    double tx = candidate.X;
                    double ty = candidate.Y;

                    if (!IsSaneCandidate(tx, ty))
                        continue;

                    var candidatePoly = orientation.CanonicalPolygon.Translate(tx, ty);

                    bool rejected = false;

                    foreach (var placed in sheet.Placed)
                    {
                        var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                            placed.X,
                            placed.Y);

                        if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                        {
                            overlapRejections++;
                            rejected = true;
                            break;
                        }
                    }

                    if (rejected)
                        continue;

                    validCandidateCount++;

                    var score = ScorePlacementCandidate(candidatePoly, sheet);

                    if (best == null || IsBetterPlacement(score, tx, ty, best))
                    {
                        best = new BestPlacement
                        {
                            X = tx,
                            Y = ty,
                            Orientation = orientation,
                            Score = score
                        };
                    }
                }
            }

            // Phase 26: interior sampling fallback. When boundary vertices all
            // fail overlap check (best == null, overlapRejections > 0), generate
            // grid points INSIDE the feasible region where there's actual clearance.
            int interiorCandidateCount = 0;
            int interiorValidCount = 0;
            bool interiorFallbackFired = false;

            if (best == null && EnableInteriorSampling && overlapRejections > 0)
            {
                interiorFallbackFired = true;
                double step = InteriorSamplingStep ??
                    Math.Max(1.0, _request.Spacing * 3.0);

                foreach (var orientation in orientations)
                {
                    if (attemptStopwatch.Elapsed > timeBudget)
                        break;

                    var ifp = InnerFitPolygon.Compute(
                        orientation,
                        _request.SheetWidth,
                        _request.SheetHeight,
                        _request.Margin);

                    if (!ifp.HasValue)
                        continue;

                    Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                    Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);

                    if (feasible.Count == 0)
                        continue;

                    var interiorPoints = SampleInteriorPoints(feasible, step);
                    if (interiorPoints.Count == 0)
                        continue;

                    interiorCandidateCount += interiorPoints.Count;

                    foreach (var pt in interiorPoints)
                    {
                        if (!IsSaneCandidate(pt.X, pt.Y))
                            continue;

                        var candidatePoly = orientation.CanonicalPolygon.Translate(pt.X, pt.Y);

                        bool rejected = false;
                        foreach (var placed in sheet.Placed)
                        {
                            var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                                placed.X, placed.Y);
                            if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                            {
                                rejected = true;
                                break;
                            }
                        }

                        if (rejected)
                            continue;

                        interiorValidCount++;

                        var score = ScorePlacementCandidate(candidatePoly, sheet);

                        if (best == null || IsBetterPlacement(score, pt.X, pt.Y, best))
                        {
                            best = new BestPlacement
                            {
                                X = pt.X,
                                Y = pt.Y,
                                Orientation = orientation,
                                Score = score
                            };
                        }
                    }
                }
            }

            if (interiorFallbackFired)
            {
                InteriorFallbackLog.Add(
                    $"Part {partIndex} sheet {sheetIdx}: boundary failed (overlapRejects={overlapRejections}), " +
                    $"interior fallback: {interiorValidCount}/{interiorCandidateCount} valid, " +
                    $"best={(best != null ? $"yes pos=({best.X:F3},{best.Y:F3})" : "no")}");
            }

            string interiorInfo = interiorFallbackFired
                ? $", interior fallback: {interiorValidCount}/{interiorCandidateCount} valid"
                : "";

            DiagnosticLog?.Invoke(
                $"  Part {partIndex} sheet {sheetIdx}: " +
                $"orient {orientationsFit}/{orientationsTried}, " +
                $"placed {sheet.Placed.Count}, " +
                $"cache {_nfpCache.Count}, " +
                $"forbidden {tForbidden}ms, " +
                $"feasible {tFeasible}ms, " +
                $"candidates {validCandidateCount}/{candidateCount}, " +
                $"sweep {tCandidateSweep}ms, " +
                $"overlapRejects {overlapRejections}{interiorInfo}, " +
                $"best={(best != null ? $"yes orient={best.Orientation.OrientationIndex} pos=({best.X:F3},{best.Y:F3}) score={best.Score.Total:F3}" : "no")}");

            if (best == null)
                return false;

            double bxLimit = BLVertexSanityFactor * _request.SheetWidth;
            double byLimit = BLVertexSanityFactor * _request.SheetHeight;

            if (Math.Abs(best.X) > bxLimit || Math.Abs(best.Y) > byLimit)
            {
                var candBBox = best.Orientation.CanonicalPolygon.BoundingBox;

                var sanityIfp = InnerFitPolygon.Compute(
                    best.Orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                string ifpStr = sanityIfp.HasValue
                    ? $"[{sanityIfp.Value.MinX:F3},{sanityIfp.Value.MinY:F3}]-[{sanityIfp.Value.MaxX:F3},{sanityIfp.Value.MaxY:F3}]"
                    : "(no-fit)";

                throw new InvalidOperationException(
                    $"NFP placement produced out-of-bounds candidate ({best.X:G6}, {best.Y:G6}) " +
                    $"for part {partIndex}, orientation {best.Orientation.OrientationIndex} " +
                    $"(rot={best.Orientation.RotationDeg:F0}{(best.Orientation.IsMirrored ? "m" : "")}, " +
                    $"src-part={best.Orientation.SourcePartIndex}), sheet {sheetIdx}. " +
                    $"Sheet is {_request.SheetWidth}x{_request.SheetHeight}; " +
                    $"limit is {BLVertexSanityFactor}x sheet dimensions. " +
                    $"Candidate canonical bbox=[{candBBox.MinX:F3},{candBBox.MinY:F3}]-[{candBBox.MaxX:F3},{candBBox.MaxY:F3}]; " +
                    $"IFP={ifpStr}. Aborting nest.");
            }

            var placedPolygon = best.Orientation.CanonicalPolygon.Translate(best.X, best.Y);

            double rotRad = best.Orientation.RotationDeg * Math.PI / 180.0;

            var sourcePoly = _request.Polygons[partIndex];
            var srcBBox = sourcePoly.BoundingBox;

            var rotatedNormalized = sourcePoly
                .MoveToOrigin()
                .RotateAround(Point2D.Origin, rotRad);

            var rotBBox = rotatedNormalized.BoundingBox;

            var step1 = Transform2D.Translation(-srcBBox.MinX, -srcBBox.MinY);
            var step2 = Transform2D.RotationDegrees(best.Orientation.RotationDeg);
            var step3 = Transform2D.Translation(-rotBBox.MinX, -rotBBox.MinY);
            var step4 = Transform2D.Translation(best.X, best.Y);

            var combined = step1.Then(step2).Then(step3).Then(step4);

            placements.Add(new PlacementResult(
                originalIndex: partIndex,
                sheet: sheetIdx,
                transform: combined,
                rotationDeg: best.Orientation.RotationDeg,
                isMirrored: best.Orientation.IsMirrored,
                sourceBBoxMinX: srcBBox.MinX,
                sourceBBoxMaxX: srcBBox.MaxX,
                placedPolygon: placedPolygon));

            sheet.Placed.Add(new PlacedItem(best.Orientation, best.X, best.Y));

            // The forbidden region depends on every placed part. Clear it after
            // each placement so the next part rebuilds against the updated sheet.
            sheet.ForbiddenByOrientation.Clear();

            return true;
        }

        // ------------------------------------------------------------------
        // Geometry steps
        // ------------------------------------------------------------------

        private Paths64 GetOrBuildForbiddenRegion(
            OrientedPart candidate,
            SheetState sheet)
        {
            if (sheet.Placed.Count == 0)
                return new Paths64();

            if (sheet.ForbiddenByOrientation.TryGetValue(
                    candidate.OrientationIndex,
                    out var cached))
            {
                return cached;
            }

            Paths64 allNfps;

            if (!ForceSerialForbiddenRegion && sheet.Placed.Count >= ParallelNfpPlacedThreshold)
            {
                allNfps = BuildForbiddenRegionParallel(candidate, sheet);
            }
            else
            {
                allNfps = BuildForbiddenRegionSerial(candidate, sheet);
            }

            // FillRule.NonZero per codebase precedent (matches Phase 21b shipped behavior).
            // This phase is a speed patch only; do not change the fill rule.
            Paths64 union = allNfps.Count == 0
                ? new Paths64()
                : Clipper.Union(allNfps, FillRule.NonZero);

            sheet.ForbiddenByOrientation[candidate.OrientationIndex] = union;
            return union;
        }

        private Paths64 BuildForbiddenRegionSerial(
            OrientedPart candidate,
            SheetState sheet)
        {
            var allNfps = new Paths64();

            foreach (var placed in sheet.Placed)
            {
                var nfp = _nfpCache.Get(placed.Orientation, candidate);

                foreach (var nfpPoly in nfp)
                {
                    var translated = nfpPoly.Translate(placed.X, placed.Y);
                    allNfps.Add(ClipperConvert.ToPath64(translated));
                }
            }

            return allNfps;
        }

        private Paths64 BuildForbiddenRegionParallel(
            OrientedPart candidate,
            SheetState sheet)
        {
            int placedCount = sheet.Placed.Count;
            var perPlacedPaths = new Paths64[placedCount];

            int maxWorkers = GetMaxNfpParallelism();

            var options = new ParallelOptions
            {
                MaxDegreeOfParallelism = maxWorkers
            };

            Parallel.For(
                0,
                placedCount,
                options,
                i =>
                {
                    var placed = sheet.Placed[i];
                    var localPaths = new Paths64();

                    var nfp = _nfpCache.Get(placed.Orientation, candidate);

                    foreach (var nfpPoly in nfp)
                    {
                        var translated = nfpPoly.Translate(placed.X, placed.Y);
                        localPaths.Add(ClipperConvert.ToPath64(translated));
                    }

                    perPlacedPaths[i] = localPaths;
                });

            // Merge in deterministic placed-order after the parallel work completes.
            var allNfps = new Paths64();

            for (int i = 0; i < perPlacedPaths.Length; i++)
            {
                var local = perPlacedPaths[i];
                if (local == null || local.Count == 0)
                    continue;

                for (int p = 0; p < local.Count; p++)
                    allNfps.Add(local[p]);
            }

            return allNfps;
        }

        private static int GetMaxNfpParallelism()
        {
            int cpu = Environment.ProcessorCount;
            int workers = Math.Max(1, cpu - 1);
            return Math.Min(MaxParallelNfpWorkers, workers);
        }

        private Paths64 ComputeFeasibleRegion(BoundingBox2D ifp, Paths64 forbidden)
        {
            var ifpPath = new Path64(4);

            long minX = (long)(ifp.MinX * ClipperConvert.Scale);
            long minY = (long)(ifp.MinY * ClipperConvert.Scale);
            long maxX = (long)(ifp.MaxX * ClipperConvert.Scale);
            long maxY = (long)(ifp.MaxY * ClipperConvert.Scale);

            // CCW rectangle: BL, BR, TR, TL.
            ifpPath.Add(new Point64(minX, minY));
            ifpPath.Add(new Point64(maxX, minY));
            ifpPath.Add(new Point64(maxX, maxY));
            ifpPath.Add(new Point64(minX, maxY));

            var subject = new Paths64 { ifpPath };

            if (forbidden.Count == 0)
                return subject;

            // FillRule.NonZero matches the codebase pattern from OverlapChecker.
            return Clipper.Difference(subject, forbidden, FillRule.NonZero);
        }

        private List<(double X, double Y)> FindCandidateVertices(
            Paths64 feasible,
            int maxCount)
        {
            var result = new List<(double X, double Y)>();

            foreach (var path in feasible)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    double x = path[i].X / ClipperConvert.Scale;
                    double y = path[i].Y / ClipperConvert.Scale;

                    bool duplicate = false;

                    for (int j = 0; j < result.Count; j++)
                    {
                        if (Math.Abs(result[j].X - x) <= CandidateDuplicateTolerance &&
                            Math.Abs(result[j].Y - y) <= CandidateDuplicateTolerance)
                        {
                            duplicate = true;
                            break;
                        }
                    }

                    if (!duplicate)
                        result.Add((x, y));
                }
            }

            // Keep the lower-left-biased candidates first so we do not spend
            // time scoring thousands of far-away vertices on complicated sheets.
            result.Sort((a, b) =>
            {
                int yCmp = a.Y.CompareTo(b.Y);
                if (yCmp != 0) return yCmp;
                return a.X.CompareTo(b.X);
            });

            if (maxCount > 0 && result.Count > maxCount)
                result.RemoveRange(maxCount, result.Count - maxCount);

            return result;
        }

        private bool IsSaneCandidate(double x, double y)
        {
            double bxLimit = BLVertexSanityFactor * _request.SheetWidth;
            double byLimit = BLVertexSanityFactor * _request.SheetHeight;

            if (double.IsNaN(x) || double.IsNaN(y)) return false;
            if (double.IsInfinity(x) || double.IsInfinity(y)) return false;

            return Math.Abs(x) <= bxLimit && Math.Abs(y) <= byLimit;
        }

        private static void FlushInteriorFallbackLog()
        {
            if (InteriorFallbackLog.Count == 0) return;

            try
            {
                var lines = new List<string>
                {
                    $"Phase 26 interior fallback log written at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                    ""
                };
                lines.AddRange(InteriorFallbackLog);
                lines.Add("");
                lines.Add($"Total interior fallback events: {InteriorFallbackLog.Count}");
                File.WriteAllLines(InteriorFallbackLogPath, lines);
            }
            catch { /* best effort */ }
        }

        private static List<(double X, double Y)> SampleInteriorPoints(
            Paths64 feasible,
            double step)
        {
            if (feasible.Count == 0 || step <= 0)
                return new List<(double, double)>();

            long scaledStep = (long)(step * ClipperConvert.Scale);
            if (scaledStep < 1) scaledStep = 1;

            long minX = long.MaxValue, minY = long.MaxValue;
            long maxX = long.MinValue, maxY = long.MinValue;

            foreach (var path in feasible)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    if (path[i].X < minX) minX = path[i].X;
                    if (path[i].Y < minY) minY = path[i].Y;
                    if (path[i].X > maxX) maxX = path[i].X;
                    if (path[i].Y > maxY) maxY = path[i].Y;
                }
            }

            if (minX >= maxX || minY >= maxY)
                return new List<(double, double)>();

            long halfStep = scaledStep / 2;
            var result = new List<(double X, double Y)>();

            for (long gy = minY + halfStep; gy <= maxY; gy += scaledStep)
            {
                for (long gx = minX + halfStep; gx <= maxX; gx += scaledStep)
                {
                    var pt = new Point64(gx, gy);

                    if (IsInsideFeasibleRegion(pt, feasible))
                    {
                        result.Add((gx / ClipperConvert.Scale, gy / ClipperConvert.Scale));
                    }

                    if (result.Count >= MaxInteriorCandidatesPerOrientation)
                        break;
                }

                if (result.Count >= MaxInteriorCandidatesPerOrientation)
                    break;
            }

            return result;
        }

        private static bool IsInsideFeasibleRegion(Point64 pt, Paths64 feasible)
        {
            int windingSum = 0;

            foreach (var path in feasible)
            {
                var pip = Clipper.PointInPolygon(pt, path);
                if (pip == PointInPolygonResult.IsOn)
                    return false;
                if (pip == PointInPolygonResult.IsInside)
                {
                    double area = Clipper.Area(path);
                    if (area > 0) windingSum++;
                    else windingSum--;
                }
            }

            return windingSum > 0;
        }


        // ------------------------------------------------------------------
        // Phase 27: single-sheet constrained beam search
        // ------------------------------------------------------------------

        private static readonly string BeamLogPath =
            Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory),
                "phase27_beam.txt");

        private struct ScoredCandidate
        {
            public OrientedPart Orientation;
            public double X;
            public double Y;
            public PlacementScore Score;
        }

        private int CountValidPlacements(
            int partIndex,
            SheetState sheet,
            int cap)
        {
            int count = 0;
            var orientations = _orientationsByPart[partIndex];

            foreach (var orientation in orientations)
            {
                var ifp = InnerFitPolygon.Compute(
                    orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                if (!ifp.HasValue)
                    continue;

                Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);

                if (feasible.Count == 0)
                    continue;

                var candidates = FindCandidateVertices(feasible, BeamMaxCandidates);

                foreach (var cand in candidates)
                {
                    if (!IsSaneCandidate(cand.X, cand.Y))
                        continue;

                    var candidatePoly = orientation.CanonicalPolygon.Translate(cand.X, cand.Y);

                    bool rejected = false;
                    foreach (var placed in sheet.Placed)
                    {
                        var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                            placed.X, placed.Y);
                        if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                        {
                            rejected = true;
                            break;
                        }
                    }

                    if (!rejected)
                    {
                        count++;
                        if (count >= cap) return count;
                    }
                }

                if (EnableInteriorSampling)
                {
                    double step = InteriorSamplingStep ??
                        Math.Max(1.0, _request.Spacing * 3.0);

                    var interiorPoints = SampleInteriorPoints(feasible, step);

                    foreach (var pt in interiorPoints)
                    {
                        if (!IsSaneCandidate(pt.X, pt.Y))
                            continue;

                        var candidatePoly = orientation.CanonicalPolygon.Translate(pt.X, pt.Y);

                        bool rejected = false;
                        foreach (var placed in sheet.Placed)
                        {
                            var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                                placed.X, placed.Y);
                            if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                            {
                                rejected = true;
                                break;
                            }
                        }

                        if (!rejected)
                        {
                            count++;
                            if (count >= cap) return count;
                        }
                    }
                }
            }

            return count;
        }

        private bool HasAnyValidPlacement(
            int partIndex,
            SheetState sheet)
        {
            return CountValidPlacements(partIndex, sheet, 1) > 0;
        }

        private List<ScoredCandidate> GenerateTopKCandidates(
            int partIndex,
            List<OrientedPart> orientations,
            SheetState sheet,
            int maxCandidates,
            int topK,
            out int boundaryValidCount,
            out int interiorValidCount)
        {
            var all = new List<ScoredCandidate>();
            boundaryValidCount = 0;
            interiorValidCount = 0;

            foreach (var orientation in orientations)
            {
                var ifp = InnerFitPolygon.Compute(
                    orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                if (!ifp.HasValue)
                    continue;

                Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);

                if (feasible.Count == 0)
                    continue;

                // Boundary vertex candidates.
                var candidates = FindCandidateVertices(feasible, maxCandidates);

                foreach (var cand in candidates)
                {
                    if (!IsSaneCandidate(cand.X, cand.Y))
                        continue;

                    var candidatePoly = orientation.CanonicalPolygon.Translate(cand.X, cand.Y);

                    bool rejected = false;
                    foreach (var placed in sheet.Placed)
                    {
                        var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                            placed.X, placed.Y);
                        if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                        {
                            rejected = true;
                            break;
                        }
                    }

                    if (rejected)
                        continue;

                    boundaryValidCount++;
                    var score = ScorePlacementCandidate(candidatePoly, sheet);
                    all.Add(new ScoredCandidate
                    {
                        Orientation = orientation,
                        X = cand.X,
                        Y = cand.Y,
                        Score = score
                    });
                }

                // Phase 28.0: interior sampling for beam candidate generation.
                // Same logic as Phase 26's fallback in TryPlaceOnSheet, but
                // runs unconditionally (not as a fallback) to maximize the
                // candidate pool before K-truncation.
                if (EnableInteriorSampling)
                {
                    double step = InteriorSamplingStep ??
                        Math.Max(1.0, _request.Spacing * 3.0);

                    var interiorPoints = SampleInteriorPoints(feasible, step);

                    foreach (var pt in interiorPoints)
                    {
                        if (!IsSaneCandidate(pt.X, pt.Y))
                            continue;

                        var candidatePoly = orientation.CanonicalPolygon.Translate(pt.X, pt.Y);

                        bool rejected = false;
                        foreach (var placed in sheet.Placed)
                        {
                            var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                                placed.X, placed.Y);
                            if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                            {
                                rejected = true;
                                break;
                            }
                        }

                        if (rejected)
                            continue;

                        interiorValidCount++;
                        var score = ScorePlacementCandidate(candidatePoly, sheet);
                        all.Add(new ScoredCandidate
                        {
                            Orientation = orientation,
                            X = pt.X,
                            Y = pt.Y,
                            Score = score
                        });
                    }
                }
            }

            // Phase 28.1.5: total-order sort. List.Sort (introsort) is
            // unstable, so any ties at Score.Total would leave the order of
            // distinct candidates undefined and break byte-identical output.
            // The ladder below is provably total for distinct candidates:
            // two candidates with identical orientation index AND identical
            // X AND identical Y (full double precision) are the same physical
            // placement, so comparing equal is correct.
            all.Sort((a, b) =>
            {
                int cmp = a.Score.Total.CompareTo(b.Score.Total);
                if (cmp != 0) return cmp;
                cmp = a.Score.UsedArea.CompareTo(b.Score.UsedArea);
                if (cmp != 0) return cmp;
                cmp = a.Score.UsedTop.CompareTo(b.Score.UsedTop);
                if (cmp != 0) return cmp;
                cmp = a.Score.UsedRight.CompareTo(b.Score.UsedRight);
                if (cmp != 0) return cmp;
                cmp = a.Orientation.OrientationIndex.CompareTo(b.Orientation.OrientationIndex);
                if (cmp != 0) return cmp;
                cmp = a.X.CompareTo(b.X);
                if (cmp != 0) return cmp;
                return a.Y.CompareTo(b.Y);
            });
            if (all.Count > topK)
                all.RemoveRange(topK, all.Count - topK);

            return all;
        }

        private sealed class BeamState
        {
            public SheetState Sheet;
            public List<PlacementResult> Placements;
            public double TotalScore;
            public double MaxUsedTop;
            public double MaxUsedRight;
        }

        public NestResult TrySingleSheetBeamPack(
            IReadOnlyList<int> partOrder,
            TimeSpan budget,
            int K,
            int B)
        {
            if (partOrder == null || partOrder.Count == 0)
                return null;

            // Phase 28.2: mutable copy of order for MRV reordering.
            var mutableOrder = new List<int>(partOrder);

            var sw = Stopwatch.StartNew();
            var log = new List<string>
            {
                $"Phase 27 beam search run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                $"Parts: {mutableOrder.Count}, B={B}, K={K}, maxCandidates={BeamMaxCandidates}, budget={budget.TotalSeconds:F0}s",
                $"Order: {string.Join(",", mutableOrder)}",
                ""
            };

            var initialState = new BeamState
            {
                Sheet = new SheetState(),
                Placements = new List<PlacementResult>(),
                TotalScore = 0,
                MaxUsedTop = 0,
                MaxUsedRight = 0
            };

            var beam = new List<BeamState> { initialState };

            for (int step = 0; step < mutableOrder.Count; step++)
            {
                if (sw.Elapsed >= budget)
                {
                    log.Add($"Part {mutableOrder[step]} (step {step}): time budget exceeded at {sw.ElapsedMilliseconds}ms. Aborting.");
                    FlushBeamLog(log);
                    return null;
                }

                // Phase 28.2: MRV reordering within Irregular tier.
                if (IrregularPartIndices != null &&
                    IrregularPartIndices.Contains(mutableOrder[step]))
                {
                    // Count unplaced Irregulars from this step onward.
                    var unplacedIrregulars = new List<int>();
                    for (int j = step; j < mutableOrder.Count; j++)
                    {
                        if (IrregularPartIndices.Contains(mutableOrder[j]))
                            unplacedIrregulars.Add(mutableOrder[j]);
                    }

                    if (unplacedIrregulars.Count >= 2 && beam.Count > 0)
                    {
                        const int mrvCap = 1000;
                        int bestIdx = unplacedIrregulars[0];
                        int bestMinCount = int.MaxValue;
                        double bestBBoxArea = 0;
                        var mrvDetails = new List<string>();

                        foreach (int ui in unplacedIrregulars)
                        {
                            int minCount = int.MaxValue;
                            foreach (var state in beam)
                            {
                                int c = CountValidPlacements(ui, state.Sheet, mrvCap);
                                if (c < minCount) minCount = c;
                            }

                            var bb = _request.Polygons[ui].BoundingBox;
                            double bboxArea = bb.Width * bb.Height;
                            mrvDetails.Add($"{ui}={minCount}");

                            if (minCount < bestMinCount ||
                                (minCount == bestMinCount && bboxArea > bestBBoxArea))
                            {
                                bestIdx = ui;
                                bestMinCount = minCount;
                                bestBBoxArea = bboxArea;
                            }
                        }

                        log.Add($"Part {mutableOrder[step]} (step {step}): MRV evaluated [{string.Join(", ", mrvDetails)}], picked Part {bestIdx}");

                        if (bestIdx != mutableOrder[step])
                        {
                            int swapFrom = mutableOrder.IndexOf(bestIdx);
                            if (swapFrom > step)
                            {
                                mutableOrder.RemoveAt(swapFrom);
                                mutableOrder.Insert(step, bestIdx);
                            }
                        }
                    }
                }

                int partIndex = mutableOrder[step];
                var orientations = _orientationsByPart[partIndex];
                var nextBeam = new List<BeamState>();

                // Build set of already-placed indices for future-domain check.
                var placedIndicesSet = new HashSet<int>();
                if (beam.Count > 0)
                {
                    foreach (var pp in beam[0].Placements)
                        placedIndicesSet.Add(pp.OriginalIndex);
                }
                placedIndicesSet.Add(partIndex);

                // Identify unplaced critical parts for future-domain pruning.
                List<int> unplacedCriticals = null;
                if (CriticalPartIndices != null && CriticalPartIndices.Count > 0)
                {
                    unplacedCriticals = new List<int>();
                    foreach (int ci in CriticalPartIndices)
                    {
                        if (!placedIndicesSet.Contains(ci))
                            unplacedCriticals.Add(ci);
                    }
                    if (unplacedCriticals.Count == 0)
                        unplacedCriticals = null;
                }

                int stateIdx = 0;
                int prunedByFutureDomain = 0;

                foreach (var state in beam)
                {
                    var topK = GenerateTopKCandidates(
                        partIndex, orientations, state.Sheet,
                        BeamMaxCandidates, K,
                        out int bValid, out int iValid);

                    foreach (var cand in topK)
                    {
                        var newSheet = state.Sheet.Clone();
                        newSheet.Placed.Add(new PlacedItem(cand.Orientation, cand.X, cand.Y));
                        newSheet.ForbiddenByOrientation.Clear();

                        // Phase 28.1: future-domain pruning. Check that each
                        // unplaced critical part still has at least one legal
                        // placement in this candidate state.
                        bool prunedByFD = false;
                        if (unplacedCriticals != null)
                        {
                            foreach (int ci in unplacedCriticals)
                            {
                                if (!HasAnyValidPlacement(ci, newSheet))
                                {
                                    prunedByFutureDomain++;
                                    prunedByFD = true;
                                    log.Add($"  Part {partIndex} state {stateIdx} candidate ({cand.X:F1},{cand.Y:F1}): " +
                                            $"PRUNED — critical Part {ci} has 0 legal placements");
                                    break;
                                }
                            }
                        }

                        if (prunedByFD)
                            continue;

                        var placedPoly = cand.Orientation.CanonicalPolygon.Translate(cand.X, cand.Y);

                        double rotRad = cand.Orientation.RotationDeg * Math.PI / 180.0;
                        var sourcePoly = _request.Polygons[partIndex];
                        var srcBBox = sourcePoly.BoundingBox;
                        var rotatedNorm = sourcePoly.MoveToOrigin();
                        if (Math.Abs(cand.Orientation.RotationDeg) > 0.01)
                            rotatedNorm = rotatedNorm.RotateAround(Point2D.Origin, rotRad);
                        var rotBBox = rotatedNorm.BoundingBox;

                        var step1 = Transform2D.Translation(-srcBBox.MinX, -srcBBox.MinY);
                        var step2 = Transform2D.RotationDegrees(cand.Orientation.RotationDeg);
                        var step3 = Transform2D.Translation(-rotBBox.MinX, -rotBBox.MinY);
                        var step4 = Transform2D.Translation(cand.X, cand.Y);
                        var combined = step1.Then(step2).Then(step3).Then(step4);

                        var newPlacements = new List<PlacementResult>(state.Placements);
                        newPlacements.Add(new PlacementResult(
                            originalIndex: partIndex,
                            sheet: 0,
                            transform: combined,
                            rotationDeg: cand.Orientation.RotationDeg,
                            isMirrored: cand.Orientation.IsMirrored,
                            sourceBBoxMinX: srcBBox.MinX,
                            sourceBBoxMaxX: srcBBox.MaxX,
                            placedPolygon: placedPoly));

                        var placedBox = placedPoly.BoundingBox;
                        double newMaxTop = Math.Max(state.MaxUsedTop, placedBox.MaxY);
                        double newMaxRight = Math.Max(state.MaxUsedRight, placedBox.MaxX);

                        nextBeam.Add(new BeamState
                        {
                            Sheet = newSheet,
                            Placements = newPlacements,
                            TotalScore = state.TotalScore + cand.Score.Total,
                            MaxUsedTop = newMaxTop,
                            MaxUsedRight = newMaxRight
                        });
                    }

                    if (EnableInteriorSampling && (bValid > 0 || iValid > 0))
                    {
                        log.Add($"  Part {partIndex} state {stateIdx}: boundary={bValid} valid, interior={iValid} valid, topK={topK.Count}");
                    }
                    stateIdx++;
                }

                if (nextBeam.Count == 0)
                {
                    log.Add($"Part {partIndex} (step {step}): beam dropped to zero — no state could place on single sheet. Aborting.");
                    FlushBeamLog(log);
                    return null;
                }

                // Phase 28.1.5: total-order sort. After the three score keys,
                // tiebreak on the placement sequence (OriginalIndex, X, Y of
                // each placement in placement order, compared lexically). Two
                // states with identical placements in identical positions are
                // genuinely the same state and may compare equal — that's
                // correct, not a determinism leak.
                nextBeam.Sort((a, b) =>
                {
                    int cmp = a.TotalScore.CompareTo(b.TotalScore);
                    if (cmp != 0) return cmp;
                    cmp = a.MaxUsedTop.CompareTo(b.MaxUsedTop);
                    if (cmp != 0) return cmp;
                    cmp = a.MaxUsedRight.CompareTo(b.MaxUsedRight);
                    if (cmp != 0) return cmp;
                    return ComparePlacementSequence(a.Placements, b.Placements);
                });

                if (nextBeam.Count > B)
                    nextBeam.RemoveRange(B, nextBeam.Count - B);

                string fdInfo = prunedByFutureDomain > 0 ? $" (futureDomain={prunedByFutureDomain})" : "";
                log.Add($"Part {partIndex} (step {step}): beam {beam.Count} -> branches {beam.Count * K} -> survived {nextBeam.Count}{fdInfo}, " +
                        $"best score {nextBeam[0].TotalScore:F1}, elapsed {sw.ElapsedMilliseconds}ms");

                beam = nextBeam;
            }

            var best = beam[0];
            log.Add("");
            log.Add($"Phase 27: beam search succeeded — all {mutableOrder.Count} parts on 1 sheet. " +
                    $"Total score {best.TotalScore:F1}, elapsed {sw.ElapsedMilliseconds}ms.");
            FlushBeamLog(log);

            return new NestResult(1, best.Placements, new List<int>());
        }

        private static void FlushBeamLog(List<string> lines)
        {
            try { File.WriteAllLines(BeamLogPath, lines); }
            catch { /* best effort */ }
        }

        // Phase 28.1.5: deterministic lexical comparison of two placement
        // sequences. Compares element-by-element on (OriginalIndex, placed
        // bbox MinX, placed bbox MinY, RotationDeg). Shorter sequence sorts
        // first if it's a prefix. Returns 0 only when the sequences are
        // genuinely identical (same parts, same positions, same order).
        private static int ComparePlacementSequence(
            List<PlacementResult> a, List<PlacementResult> b)
        {
            int n = Math.Min(a.Count, b.Count);
            for (int i = 0; i < n; i++)
            {
                int cmp = a[i].OriginalIndex.CompareTo(b[i].OriginalIndex);
                if (cmp != 0) return cmp;

                var ba = a[i].PlacedPolygon.BoundingBox;
                var bb = b[i].PlacedPolygon.BoundingBox;
                cmp = ba.MinX.CompareTo(bb.MinX);
                if (cmp != 0) return cmp;
                cmp = ba.MinY.CompareTo(bb.MinY);
                if (cmp != 0) return cmp;
                cmp = a[i].RotationDeg.CompareTo(b[i].RotationDeg);
                if (cmp != 0) return cmp;
            }
            return a.Count.CompareTo(b.Count);
        }
        // ------------------------------------------------------------------
        // Smart candidate scoring
        // ------------------------------------------------------------------

        private PlacementScore ScorePlacementCandidate(
            Polygon candidatePoly,
            SheetState sheet)
        {
            var candidateBox = ToBox(candidatePoly.BoundingBox);

            double usedRight = candidateBox.MaxX;
            double usedTop = candidateBox.MaxY;
            double usedMinX = candidateBox.MinX;
            double usedMinY = candidateBox.MinY;

            double totalPlacedArea = candidatePoly.AbsoluteArea;

            for (int i = 0; i < sheet.Placed.Count; i++)
            {
                var placed = sheet.Placed[i];
                var placedBox = GetPlacedBox(placed);

                usedRight = Math.Max(usedRight, placedBox.MaxX);
                usedTop = Math.Max(usedTop, placedBox.MaxY);
                usedMinX = Math.Min(usedMinX, placedBox.MinX);
                usedMinY = Math.Min(usedMinY, placedBox.MinY);

                totalPlacedArea += placed.Orientation.CanonicalPolygon.AbsoluteArea;
            }

            double usedWidth = Math.Max(0.0, usedRight - Math.Min(0.0, usedMinX));
            double usedHeight = Math.Max(0.0, usedTop - Math.Min(0.0, usedMinY));
            double usedArea = usedWidth * usedHeight;

            double contact =
                ComputeSheetEdgeContact(candidateBox) +
                ComputePartContact(candidateBox, sheet);

            double looseIslandPenalty = 0.0;

            if (sheet.Placed.Count > 0 && contact <= ContactTolerance)
            {
                // Penalize placements that create separated islands instead of
                // packing against existing material or the sheet edge.
                looseIslandPenalty = candidateBox.Width + candidateBox.Height;
            }

            double bottomLeftBias =
                candidateBox.MinY * BottomBiasWeight +
                candidateBox.MinX * LeftBiasWeight;

            double wasteInsideEnvelope = Math.Max(0.0, usedArea - totalPlacedArea);

            double total =
                usedArea * UsedAreaWeight +
                usedTop * UsedTopWeight +
                usedRight * UsedRightWeight +
                bottomLeftBias +
                wasteInsideEnvelope * 2.0 +
                looseIslandPenalty * LooseIslandPenaltyWeight -
                contact * ContactRewardWeight;

            return new PlacementScore
            {
                Total = total,
                UsedRight = usedRight,
                UsedTop = usedTop,
                UsedArea = usedArea,
                ContactLength = contact,
                WasteInsideEnvelope = wasteInsideEnvelope
            };
        }

        private bool IsBetterPlacement(
            PlacementScore score,
            double x,
            double y,
            BestPlacement best)
        {
            if (score.Total < best.Score.Total - 1e-9)
                return true;

            if (Math.Abs(score.Total - best.Score.Total) > 1e-9)
                return false;

            if (score.UsedArea < best.Score.UsedArea - 1e-9)
                return true;

            if (Math.Abs(score.UsedArea - best.Score.UsedArea) > 1e-9)
                return false;

            if (score.UsedTop < best.Score.UsedTop - 1e-9)
                return true;

            if (Math.Abs(score.UsedTop - best.Score.UsedTop) > 1e-9)
                return false;

            if (score.UsedRight < best.Score.UsedRight - 1e-9)
                return true;

            if (Math.Abs(score.UsedRight - best.Score.UsedRight) > 1e-9)
                return false;

            if (score.ContactLength > best.Score.ContactLength + 1e-9)
                return true;

            if (Math.Abs(score.ContactLength - best.Score.ContactLength) > 1e-9)
                return false;

            if (y < best.Y - 1e-9)
                return true;

            if (Math.Abs(y - best.Y) <= 1e-9 && x < best.X - 1e-9)
                return true;

            return false;
        }

        private double ComputeSheetEdgeContact(Box2 box)
        {
            double contact = 0.0;

            double sheetMinX = _request.Margin;
            double sheetMinY = _request.Margin;
            double sheetMaxX = _request.SheetWidth - _request.Margin;
            double sheetMaxY = _request.SheetHeight - _request.Margin;

            if (Near(box.MinX, sheetMinX)) contact += box.Height;
            if (Near(box.MinY, sheetMinY)) contact += box.Width;
            if (Near(box.MaxX, sheetMaxX)) contact += box.Height;
            if (Near(box.MaxY, sheetMaxY)) contact += box.Width;

            return contact;
        }

        private double ComputePartContact(Box2 candidate, SheetState sheet)
        {
            double contact = 0.0;

            for (int i = 0; i < sheet.Placed.Count; i++)
            {
                var placed = GetPlacedBox(sheet.Placed[i]);

                if (Near(candidate.MinX, placed.MaxX))
                {
                    contact += OverlapLength(
                        candidate.MinY,
                        candidate.MaxY,
                        placed.MinY,
                        placed.MaxY);
                }

                if (Near(candidate.MaxX, placed.MinX))
                {
                    contact += OverlapLength(
                        candidate.MinY,
                        candidate.MaxY,
                        placed.MinY,
                        placed.MaxY);
                }

                if (Near(candidate.MinY, placed.MaxY))
                {
                    contact += OverlapLength(
                        candidate.MinX,
                        candidate.MaxX,
                        placed.MinX,
                        placed.MaxX);
                }

                if (Near(candidate.MaxY, placed.MinY))
                {
                    contact += OverlapLength(
                        candidate.MinX,
                        candidate.MaxX,
                        placed.MinX,
                        placed.MaxX);
                }
            }

            return contact;
        }

        private static double OverlapLength(
            double aMin,
            double aMax,
            double bMin,
            double bMax)
        {
            double lo = Math.Max(aMin, bMin);
            double hi = Math.Min(aMax, bMax);
            return Math.Max(0.0, hi - lo);
        }

        private static bool Near(double a, double b)
        {
            return Math.Abs(a - b) <= ContactTolerance;
        }

        private static Box2 ToBox(BoundingBox2D bb)
        {
            return new Box2
            {
                MinX = bb.MinX,
                MinY = bb.MinY,
                MaxX = bb.MaxX,
                MaxY = bb.MaxY
            };
        }

        private static Box2 GetPlacedBox(PlacedItem placed)
        {
            var bb = placed.Orientation.CanonicalPolygon.BoundingBox;

            return new Box2
            {
                MinX = bb.MinX + placed.X,
                MinY = bb.MinY + placed.Y,
                MaxX = bb.MaxX + placed.X,
                MaxY = bb.MaxY + placed.Y
            };
        }

        // ------------------------------------------------------------------
        // Per-call state
        // ------------------------------------------------------------------

        private sealed class SheetState
        {
            public List<PlacedItem> Placed { get; } = new List<PlacedItem>();

            public Dictionary<int, Paths64> ForbiddenByOrientation { get; }
                = new Dictionary<int, Paths64>();

            public SheetState Clone()
            {
                var copy = new SheetState();
                copy.Placed.AddRange(this.Placed);
                return copy;
            }
        }

        private readonly struct PlacedItem
        {
            public OrientedPart Orientation { get; }
            public double X { get; }
            public double Y { get; }

            public PlacedItem(OrientedPart orientation, double x, double y)
            {
                Orientation = orientation;
                X = x;
                Y = y;
            }
        }

        private sealed class BestPlacement
        {
            public double X;
            public double Y;
            public OrientedPart Orientation;
            public PlacementScore Score;
        }

        private struct PlacementScore
        {
            public double Total;
            public double UsedRight;
            public double UsedTop;
            public double UsedArea;
            public double ContactLength;
            public double WasteInsideEnvelope;
        }

        private struct Box2
        {
            public double MinX;
            public double MinY;
            public double MaxX;
            public double MaxY;

            public double Width => MaxX - MinX;
            public double Height => MaxY - MinY;
        }
    }
}
