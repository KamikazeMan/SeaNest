using System;
using System.Collections.Generic;
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

        private const int MaxCandidateVerticesPerOrientation = 96;

        // Phase 22b — exact-demand parallel NFP gathering.
        // Below this count, Parallel.For overhead can cost more than it saves.
        private const int ParallelNfpPlacedThreshold = 3;

        // Cap worker count so Rhino remains responsive and Clipper/GC pressure stays sane.
        private const int MaxParallelNfpWorkers = 8;

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
                    MaxCandidateVerticesPerOrientation);
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

            DiagnosticLog?.Invoke(
                $"  Part {partIndex} sheet {sheetIdx}: " +
                $"orient {orientationsFit}/{orientationsTried}, " +
                $"placed {sheet.Placed.Count}, " +
                $"cache {_nfpCache.Count}, " +
                $"forbidden {tForbidden}ms, " +
                $"feasible {tFeasible}ms, " +
                $"candidates {validCandidateCount}/{candidateCount}, " +
                $"sweep {tCandidateSweep}ms, " +
                $"overlapRejects {overlapRejections}, " +
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

            if (sheet.Placed.Count >= ParallelNfpPlacedThreshold)
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
