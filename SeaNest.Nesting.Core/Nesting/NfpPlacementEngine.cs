using System;
using System.Collections.Generic;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// NFP-based placement engine. Replaces BLF's bbox-corner candidate generation with
    /// feasible-region boundary walking: for each oriented part, the set of valid
    /// reference-point translations is the IFP minus the union of NFPs against already-
    /// placed parts. The bottom-left-most vertex of that feasible region is the placement.
    ///
    /// Algorithm (per part, per sheet, per orientation):
    ///   1. Compute IFP rectangle (file 6) — too-large parts are skipped here.
    ///   2. Fetch NFPs of placed parts vs the candidate orientation from <see cref="NfpCache"/>.
    ///   3. Union all NFPs via Clipper2 → forbidden region.
    ///   4. IFP rectangle MINUS forbidden region (Clipper Difference) → feasible region.
    ///   5. Sweep feasible region's boundary vertices for the BL-best (smallest Y, ties on X).
    ///   6. Across all orientations, the BL-best wins the placement.
    ///
    /// Multi-sheet overflow matches Phase 1: try existing sheets in order, open new sheet
    /// on failure, mark unplaced if a fresh sheet also rejects.
    ///
    /// Engine is single-pass and stateful per call — <see cref="PlaceAll"/> consumes a
    /// part order and returns a complete result. Simulated annealing wraps this method,
    /// calling it many times with perturbed orderings.
    ///
    /// FillRule contract:
    ///   Every Clipper2 boolean in this file uses <see cref="FillRule.Positive"/>.
    ///   Inputs to this engine are <see cref="OrientedPart.CanonicalPolygon"/>s, which
    ///   <see cref="OrientedPart.Build"/> forces to CCW (and asserts). NFP outputs from
    ///   <see cref="NoFitPolygon"/> inherit that CCW outer winding; any CW sub-loops
    ///   encode holes that Positive correctly subtracts from the union. See
    ///   <see cref="NoFitPolygon"/> for the matching FillRule rationale, and
    ///   <see cref="Overlap.OverlapChecker"/> for why the overlap-detection path
    ///   deliberately uses NonZero instead.
    /// </summary>
    public sealed class NfpPlacementEngine
    {
        private readonly NestRequest _request;
        private readonly IReadOnlyList<List<OrientedPart>> _orientationsByPart;
        private readonly NfpCache _nfpCache;
        public Action<string> DiagnosticLog { get; set; }

        /// <summary>
        /// Construct an engine bound to a single nest request. The orientation list and
        /// NFP cache are built up-front in <see cref="NestingEngine"/> and shared across
        /// all annealing iterations — only the part order changes per iteration.
        /// </summary>
        public NfpPlacementEngine(
            NestRequest request,
            IReadOnlyList<List<OrientedPart>> orientationsByPart,
            NfpCache nfpCache)
        {
            _request = request ?? throw new ArgumentNullException(nameof(request));
            _orientationsByPart = orientationsByPart ?? throw new ArgumentNullException(nameof(orientationsByPart));
            _nfpCache = nfpCache ?? throw new ArgumentNullException(nameof(nfpCache));
        }

        /// <summary>
        /// Place all parts in <paramref name="partOrder"/> using NFP-based placement.
        ///
        /// <paramref name="partOrder"/> is a list of source-part indices (i.e., indices
        /// into <see cref="NestRequest.Polygons"/>) — annealing rearranges this between
        /// calls to explore the placement space. Every source part should appear exactly
        /// once; passing a partial order will leave the omitted parts unplaced.
        /// </summary>
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
                    placed = TryPlaceOnSheet(partIndex, orientations, sheets[sheetIdx], sheetIdx, placements);
                }

                if (!placed)
                {
                    var newSheet = new SheetState();
                    sheets.Add(newSheet);
                    int newSheetIdx = sheets.Count - 1;
                    placed = TryPlaceOnSheet(partIndex, orientations, newSheet, newSheetIdx, placements);

                    if (!placed)
                    {
                        sheets.RemoveAt(newSheetIdx);
                        unplaced.Add(partIndex);
                    }
                }
            }

            return new NestResult(sheets.Count, placements, unplaced);
        }

        /// <summary>
        /// Result of one <see cref="PlaceAll"/> call. The simulated-annealing wrapper
        /// scores these and keeps the best.
        /// </summary>
        public sealed class NestResult
        {
            public int SheetCount { get; }
            public IReadOnlyList<PlacementResult> Placements { get; }
            public IReadOnlyList<int> Unplaced { get; }
            
            public NestResult(int sheetCount, IReadOnlyList<PlacementResult> placements, IReadOnlyList<int> unplaced)
            {
                SheetCount = sheetCount;
                Placements = placements;
                Unplaced = unplaced;
            }
        }

        // ------------------------------------------------------------------
        // Per-sheet placement
        // ------------------------------------------------------------------

        /// <summary>
        /// Wall-clock cap on a single TryPlaceOnSheet call. Catches Clipper2
        /// degenerate-input loops and runaway NFP computations that previously
        /// produced multi-minute hangs on real boat-part inputs. Hardcoded for
        /// now; promote to NestRequest if user-configurable becomes useful.
        /// </summary>
        private const int TryPlaceOnSheetTimeBudgetSeconds = 30;

        /// <summary>
        /// Sanity bound on the BL vertex picked from the feasible region. Catches
        /// numerical breakdowns where Clipper.Difference output included vertices
        /// far outside the IFP rectangle (we observed 132M+ inches on a 96-inch
        /// sheet during early concave-geometry testing). 10× sheet dimensions
        /// is well past any legitimate placement and well below typical garbage.
        /// </summary>
        private const double BLVertexSanityFactor = 10.0;

        /// <summary>
        /// Linear tolerance for the Phase 6d defensive in-loop overlap check.
        /// MUST match <see cref="Verification.FinalVerifier.VerifyTolerance"/> (1e-4)
        /// so the engine's pre-commit check and the verifier's post-nest check share
        /// a single verdict — anything the engine accepts here, the verifier accepts
        /// there. <see cref="Overlap.OverlapChecker.Overlaps"/> squares this internally
        /// against the intersection area, giving the 1e-8 area threshold the verifier
        /// uses. Inlined rather than referenced via FinalVerifier to keep
        /// SeaNest.Nesting.Core/Nesting free of a Verification namespace dependency.
        /// </summary>
        private const double OverlapTolerance = 1e-4;

        private bool TryPlaceOnSheet(
    int partIndex,
    List<OrientedPart> orientations,
    SheetState sheet,
    int sheetIdx,
    List<PlacementResult> placements)
        {
            BestPlacement best = null;
            int overlapRejections = 0;

            // Phase 12 (TEMPORARY): entry-state diagnostic for the
            // "part fits individually but doesn't get placed" investigation.
            // Logs sheet dims, source bbox, and the per-part orientation count
            // so we can verify the engine sees the expected enumeration. Each
            // orientation gets its own fits-on-sheet line below.
            var diagSourcePoly = _request.Polygons[partIndex];
            var diagSrcBBox = diagSourcePoly.BoundingBox;
            double diagSrcW = diagSrcBBox.MaxX - diagSrcBBox.MinX;
            double diagSrcH = diagSrcBBox.MaxY - diagSrcBBox.MinY;
            DiagnosticLog?.Invoke(
                $"  TryPlaceOnSheet entry: part={partIndex} sheet={sheetIdx} " +
                $"sheet.Placed.Count={sheet.Placed.Count} " +
                $"source bbox: {diagSrcW:F2} × {diagSrcH:F2}, " +
                $"sheet: {_request.SheetWidth:F2} × {_request.SheetHeight:F2} " +
                $"(margin {_request.Margin:F2}), " +
                $"orientations: {orientations.Count}");

            long tForbidden = 0;
            long tFeasible = 0;
            long tBlSweep = 0;
            int orientationsTried = 0;
            int orientationsFit = 0;
            var sw = new System.Diagnostics.Stopwatch();

            // Per-attempt time budget. Checked once per orientation. Both branches
            // soft-exit by breaking out of the orientation loop:
            //   - best != null: caller commits the best-so-far placement.
            //   - best == null: caller sees `return false` (line ~247) and falls
            //     through its existing same-sheet-failure path — try next existing
            //     sheet, then a fresh sheet. The budget firing means "ran out of
            //     time on this sheet," not "no placement exists anywhere."
            // The BL coordinate sanity bound (line ~257) is the only remaining
            // throw in this method and guards a true error condition.
            var attemptStopwatch = System.Diagnostics.Stopwatch.StartNew();
            var timeBudget = System.TimeSpan.FromSeconds(TryPlaceOnSheetTimeBudgetSeconds);

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
                            $"{orientationsFit} fit).");
                    }
                    else
                    {
                        DiagnosticLog?.Invoke(
                            $"  Part {partIndex} budget exceeded after " +
                            $"{attemptStopwatch.Elapsed.TotalSeconds:F1}s on sheet {sheetIdx} " +
                            $"— no placement found in time, falling through to next sheet " +
                            $"({orientationsTried}/{orientations.Count} orientations checked, " +
                            $"{orientationsFit} IFP-feasible).");
                    }
                    break;
                }

                orientationsTried++;

                // Phase 12 (TEMPORARY): per-orientation fits-on-sheet check
                // mirroring InnerFitPolygon's logic. Logs whether each
                // orientation's rotated bbox fits within the usable sheet
                // area (sheet - 2·margin in each direction) and which axis
                // failed when it doesn't. ifp.HasValue below is the
                // authoritative gate, but this log shows the precondition
                // so we can spot orientations that geometrically fit but
                // get rejected anyway, vs orientations that genuinely
                // can't fit at any position.
                var canonBBox = orientation.BBox;
                double oriW = canonBBox.MaxX - canonBBox.MinX;
                double oriH = canonBBox.MaxY - canonBBox.MinY;
                double usableW = _request.SheetWidth - 2.0 * _request.Margin;
                double usableH = _request.SheetHeight - 2.0 * _request.Margin;
                bool fitsW = oriW <= usableW;
                bool fitsH = oriH <= usableH;
                bool fitsBBox = fitsW && fitsH;
                string failReason =
                    fitsBBox ? "" :
                    (!fitsW && !fitsH) ? $" ({oriW:F2} > {usableW:F2} AND {oriH:F2} > {usableH:F2})" :
                    !fitsW ? $" ({oriW:F2} > {usableW:F2})" :
                    $" ({oriH:F2} > {usableH:F2})";
                DiagnosticLog?.Invoke(
                    $"    orient {orientation.OrientationIndex}: " +
                    $"rot {orientation.RotationDeg:F0}°, mirror={orientation.IsMirrored}, " +
                    $"bbox {oriW:F2} × {oriH:F2}, fits={fitsBBox}{failReason}");

                var ifp = InnerFitPolygon.Compute(
                    orientation, _request.SheetWidth, _request.SheetHeight, _request.Margin);
                if (!ifp.HasValue) continue;
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

                if (feasible.Count == 0) continue;

                sw.Restart();
                var blVertex = FindBottomLeftVertex(feasible);
                sw.Stop();
                tBlSweep += sw.ElapsedMilliseconds;

                if (blVertex == null) continue;

                double tx = blVertex.Value.X;
                double ty = blVertex.Value.Y;

                // Phase 6d defensive in-loop overlap check. Until the underlying NFP
                // bug is identified, verify that the BL search result doesn't actually
                // overlap any prior placement on this sheet — and reject the
                // orientation if it does. Same overlap primitive and threshold the
                // FinalVerifier uses, so anything we accept here, the verifier accepts.
                var candidatePoly = orientation.CanonicalPolygon.Translate(tx, ty);
                bool rejected = false;
                foreach (var placed in sheet.Placed)
                {
                    var priorPoly = placed.Orientation.CanonicalPolygon.Translate(placed.X, placed.Y);
                    if (Overlap.OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                    {
                        overlapRejections++;
                        DiagnosticLog?.Invoke(
                            $"  Part {partIndex} sheet {sheetIdx} orient={orientation.OrientationIndex}: " +
                            $"BL search picked ({tx:F3},{ty:F3}) — REJECTED (#{overlapRejections} for this part), " +
                            $"overlaps Part {placed.Orientation.SourcePartIndex} " +
                            $"(orient={placed.Orientation.OrientationIndex}, pos=({placed.X:F3},{placed.Y:F3})).");
                        rejected = true;
                        break;
                    }
                }
                if (rejected) continue;

                if (best == null ||
                    ty < best.Y - 1e-9 ||
                    (System.Math.Abs(ty - best.Y) < 1e-9 && tx < best.X - 1e-9))
                {
                    best = new BestPlacement
                    {
                        X = tx,
                        Y = ty,
                        Orientation = orientation
                    };
                }
            }

            DiagnosticLog?.Invoke(
    $"  Part {partIndex} sheet {sheetIdx}: " +
    $"orient {orientationsFit}/{orientationsTried}, " +
    $"placed {sheet.Placed.Count}, " +
    $"cache {_nfpCache.Count}, " +
    $"forbidden {tForbidden}ms, " +
    $"feasible {tFeasible}ms, " +
    $"BL {tBlSweep}ms, " +
    $"best={(best != null ? $"yes orient={best.Orientation.OrientationIndex} BL=({best.X:F3},{best.Y:F3})" : "no")}");



            if (best == null) return false;

            // Sanity bound on the chosen BL vertex. If Clipper.Difference produced
            // escaped vertices far outside the IFP rectangle (numerical breakdown
            // on degenerate concave Minkowski input), the BL vertex selection
            // latches onto numerical-infinity coordinates. Throw with full context
            // so we can reverse-engineer which (orientation, source-shape) pair
            // tripped the bound if it ever fires.
            double bxLimit = BLVertexSanityFactor * _request.SheetWidth;
            double byLimit = BLVertexSanityFactor * _request.SheetHeight;
            if (System.Math.Abs(best.X) > bxLimit || System.Math.Abs(best.Y) > byLimit)
            {
                var candBBox = best.Orientation.CanonicalPolygon.BoundingBox;
                var sanityIfp = InnerFitPolygon.Compute(
                    best.Orientation, _request.SheetWidth, _request.SheetHeight, _request.Margin);
                string ifpStr = sanityIfp.HasValue
                    ? $"[{sanityIfp.Value.MinX:F3},{sanityIfp.Value.MinY:F3}]-[{sanityIfp.Value.MaxX:F3},{sanityIfp.Value.MaxY:F3}]"
                    : "(no-fit)";

                throw new System.InvalidOperationException(
                    $"NFP placement produced out-of-bounds BL vertex ({best.X:G6}, {best.Y:G6}) " +
                    $"for part {partIndex}, orientation {best.Orientation.OrientationIndex} " +
                    $"(rot={best.Orientation.RotationDeg:F0}{(best.Orientation.IsMirrored ? "m" : "")}, " +
                    $"src-part={best.Orientation.SourcePartIndex}), sheet {sheetIdx}. " +
                    $"Sheet is {_request.SheetWidth}×{_request.SheetHeight}; " +
                    $"limit is {BLVertexSanityFactor}× sheet dimensions. " +
                    $"Candidate canonical bbox=[{candBBox.MinX:F3},{candBBox.MinY:F3}]-[{candBBox.MaxX:F3},{candBBox.MaxY:F3}]; " +
                    $"IFP={ifpStr}. " +
                    $"Likely concave geometry triggering CW-handling breakdown (check NFP anomalies emitted earlier). " +
                    $"Aborting nest.");
            }

            var placedPolygon = best.Orientation.CanonicalPolygon.Translate(best.X, best.Y);

            var rotRad = best.Orientation.RotationDeg * System.Math.PI / 180.0;
            var sourcePoly = _request.Polygons[partIndex];
            var srcBBox = sourcePoly.BoundingBox;

            // Use normalized (bbox-at-origin) source for the post-rotation bbox computation.
            // Mismatch with BLF here was a latent bug — surfaced when Phase 7b began rendering inner loops via Transform.
            var rotatedNormalized = sourcePoly.MoveToOrigin().RotateAround(Point2D.Origin, rotRad);
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

            var newPlaced = new PlacedItem(best.Orientation, best.X, best.Y);
            sheet.Placed.Add(newPlaced);

            // Invalidate the per-attempt forbidden-region cache. The next
            // TryPlaceOnSheet call rebuilds entries lazily as it queries each
            // candidate orientation; orientations that the engine never queries
            // never get computed. See GetOrBuildForbiddenRegion for the cache-
            // miss build path.
            sheet.ForbiddenByOrientation.Clear();

            return true;
        }

        // ------------------------------------------------------------------
        // Geometry steps
        // ------------------------------------------------------------------

        /// <summary>
        /// Get the forbidden region for this candidate orientation on this sheet,
        /// building it on demand from the currently-placed parts and the NFP cache.
        ///
        /// Sheet-level cache (<see cref="SheetState.ForbiddenByOrientation"/>) holds
        /// results for the duration of a single placement attempt batch — repeated
        /// queries for the same candidate orientation within one TryPlaceOnSheet call
        /// hit the cache. After every placement, <see cref="TryPlaceOnSheet"/> clears
        /// the sheet cache so the next attempt rebuilds against the new placed-parts
        /// set. Orientations the engine never queries are never computed; only the
        /// orientations of the currently-placing candidate ever produce work.
        ///
        /// Each cached NFP from <see cref="_nfpCache"/> is in translation space relative
        /// to the placed part's reference point (its bbox-min); to express it in the
        /// candidate orientation's translation space, we translate by the placed part's
        /// placement (X, Y) before unioning.
        /// </summary>
        private Paths64 GetOrBuildForbiddenRegion(
    OrientedPart candidate,
    SheetState sheet)
        {
            if (sheet.Placed.Count == 0)
                return new Paths64();

            if (sheet.ForbiddenByOrientation.TryGetValue(candidate.OrientationIndex, out var cached))
                return cached;

            // Cache miss — build the cumulative forbidden region from sheet.Placed
            // by fetching each (placed orientation, candidate) NFP from _nfpCache,
            // translating by the placed part's placement, and unioning.
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

            Paths64 union = allNfps.Count == 0
                ? new Paths64()
                : Clipper.Union(allNfps, FillRule.Positive);

            sheet.ForbiddenByOrientation[candidate.OrientationIndex] = union;
            return union;
        }

        /// <summary>
        /// Feasible region = IFP rectangle MINUS forbidden region. Clipper Difference
        /// in int64 space.
        /// </summary>
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

            return Clipper.Difference(subject, forbidden, FillRule.Positive);
        }

        /// <summary>
        /// Find the bottom-left vertex among all feasible-region vertices. Smallest Y
        /// wins; ties broken by smallest X. Coordinates returned in model units.
        ///
        /// Note: the BL-most point of a polygonal region is always at a vertex (the
        /// boundary is a straight-line polygon, so the minimum of any linear function
        /// over it is attained at an extreme point). No interior or edge sweep needed.
        /// </summary>
        private (double X, double Y)? FindBottomLeftVertex(Paths64 feasible)
        {
            double bestY = double.MaxValue;
            double bestX = double.MaxValue;
            bool found = false;

            foreach (var path in feasible)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    double x = path[i].X / ClipperConvert.Scale;
                    double y = path[i].Y / ClipperConvert.Scale;

                    if (!found ||
                        y < bestY - 1e-9 ||
                        (Math.Abs(y - bestY) < 1e-9 && x < bestX - 1e-9))
                    {
                        bestX = x;
                        bestY = y;
                        found = true;
                    }
                }
            }

            return found ? ((double, double)?)(bestX, bestY) : null;
        }

        // ------------------------------------------------------------------
        // Per-call state
        // ------------------------------------------------------------------

        private sealed class SheetState
        {
            public List<PlacedItem> Placed { get; } = new List<PlacedItem>();

            /// <summary>
            /// Per-attempt cache of forbidden regions keyed by candidate orientation index.
            /// Populated lazily by <see cref="GetOrBuildForbiddenRegion"/> on first query
            /// for an orientation; cleared by <see cref="TryPlaceOnSheet"/> immediately
            /// after every successful placement so the next attempt rebuilds against the
            /// new placed-parts set. Orientations the engine never queries during an
            /// attempt are never computed — that's the win over the previous eager-on-
            /// every-orientation pre-population.
            /// </summary>
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
        }
    }
}