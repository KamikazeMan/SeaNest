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

        private bool TryPlaceOnSheet(
    int partIndex,
    List<OrientedPart> orientations,
    SheetState sheet,
    int sheetIdx,
    List<PlacementResult> placements)
        {
            BestPlacement best = null;

            long tNfpFetch = 0;
            long tForbidden = 0;
            long tFeasible = 0;
            long tBlSweep = 0;
            int orientationsTried = 0;
            int orientationsFit = 0;
            var sw = new System.Diagnostics.Stopwatch();

            foreach (var orientation in orientations)
            {
                orientationsTried++;

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

                // Per-attempt diagnostic — drives the engine-vs-verifier overlap
                // diagnosis. Emits exactly when DiagnosticLog is wired AND there is
                // at least one part already placed on this sheet (the case we're
                // hunting). For the first part on a sheet, forbidden is trivially
                // empty by design and the diagnostic noise isn't useful.
                if (DiagnosticLog != null && sheet.Placed.Count > 0)
                {
                    DescribePaths64(forbidden, out int forbPaths, out int forbVerts,
                        out double forbAreaCcw, out double forbAreaCw,
                        out double forbMinX, out double forbMinY,
                        out double forbMaxX, out double forbMaxY);
                    DescribePaths64(feasible, out int feasPaths, out int feasVerts,
                        out double feasAreaCcw, out double feasAreaCw,
                        out double feasMinX, out double feasMinY,
                        out double feasMaxX, out double feasMaxY);

                    // Decisive test: is the IFP min corner geometrically inside the
                    // forbidden region? If yes, Clipper.Difference must have clipped it
                    // and the BL vertex cannot be (IFP.MinX, IFP.MinY) — UNLESS Clipper
                    // is leaving it intact (Clipper bug or fill-rule edge case).
                    // If no, the forbidden region is mis-positioned and the IFP corner
                    // legitimately survives the carve.
                    string ifpCornerVerdict = ForbiddenContainsPoint(forbidden, ifpBox.MinX, ifpBox.MinY)
                        ? "INSIDE-forbidden"
                        : "OUTSIDE-forbidden";

                    DiagnosticLog.Invoke(
                        $"  Attempt p{partIndex}/o{orientation.OrientationIndex} sheet{sheetIdx} " +
                        $"rot{orientation.RotationDeg:F0}{(orientation.IsMirrored ? "m" : "")} " +
                        $"IFP=[{ifpBox.MinX:F3},{ifpBox.MinY:F3}]-[{ifpBox.MaxX:F3},{ifpBox.MaxY:F3}] " +
                        $"forb={{paths={forbPaths}, verts={forbVerts}, areaCCW={forbAreaCcw:G6}, areaCW={forbAreaCw:G6}, " +
                        $"bbox=[{forbMinX:F3},{forbMinY:F3}]-[{forbMaxX:F3},{forbMaxY:F3}]}} " +
                        $"feas={{paths={feasPaths}, verts={feasVerts}, areaCCW={feasAreaCcw:G6}, areaCW={feasAreaCw:G6}, " +
                        $"bbox=[{feasMinX:F3},{feasMinY:F3}]-[{feasMaxX:F3},{feasMaxY:F3}]}} " +
                        $"IFPcorner={ifpCornerVerdict}");
                }

                if (feasible.Count == 0) continue;

                sw.Restart();
                var blVertex = FindBottomLeftVertex(feasible);
                sw.Stop();
                tBlSweep += sw.ElapsedMilliseconds;

                if (blVertex == null) continue;

                double tx = blVertex.Value.X;
                double ty = blVertex.Value.Y;

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

            var placedPolygon = best.Orientation.CanonicalPolygon.Translate(best.X, best.Y);

            // Reference-point sanity check at placement commit. Three values that
            // should agree on the part's anchor:
            //   canon.BBox.Min  — should be (0, 0) by OrientedPart.Build's MoveToOrigin.
            //   chosen BL       — engine's pick from the feasible region's vertices.
            //   placed.BBox.Min — should equal chosen BL after Translate(X, Y).
            // If chosen BL differs from placed.BBox.Min by anything other than (0, 0),
            // the canonical polygon's bbox-min isn't actually at origin and the entire
            // placement convention is off by that delta — which is exactly the
            // reference-point bug we're hunting.
            if (DiagnosticLog != null && sheet.Placed.Count > 0)
            {
                var canonBBox = best.Orientation.CanonicalPolygon.BoundingBox;
                var placedBBox = placedPolygon.BoundingBox;
                DiagnosticLog.Invoke(
                    $"  Commit p{partIndex}/o{best.Orientation.OrientationIndex} sheet{sheetIdx} " +
                    $"BL=({best.X:F4},{best.Y:F4}) " +
                    $"canon.BBox.Min=({canonBBox.MinX:F4},{canonBBox.MinY:F4}) " +
                    $"placed.BBox.Min=({placedBBox.MinX:F4},{placedBBox.MinY:F4}) " +
                    $"deltaBLvsPlaced=({best.X - placedBBox.MinX:F4},{best.Y - placedBBox.MinY:F4})");
            }

            var rotRad = best.Orientation.RotationDeg * System.Math.PI / 180.0;
            var sourcePoly = _request.Polygons[partIndex];
            var srcBBox = sourcePoly.BoundingBox;

            var rotatedSource = sourcePoly.RotateAround(Point2D.Origin, rotRad);
            var rotBBox = rotatedSource.BoundingBox;

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
                placedPolygon: placedPolygon));

            var newPlaced = new PlacedItem(best.Orientation, best.X, best.Y);
            sheet.Placed.Add(newPlaced);
            UpdateForbiddenAfterPlacement(newPlaced, sheet);
            return true;
        }

        // ------------------------------------------------------------------
        // Geometry steps
        // ------------------------------------------------------------------

        /// <summary>
        /// Get the cumulative forbidden region for this candidate orientation on this sheet.
        /// Returns the cached union if available, otherwise builds it from scratch from
        /// every already-placed part's NFP against this candidate orientation.
        ///
        /// Each cached NFP is in translation space relative to the placed part's reference
        /// point; to express it in the candidate orientation's translation space, we
        /// translate by the placed part's placement (X, Y) before unioning.
        ///
        /// O(1) work per call after the orientation is first seen — see <see cref="UpdateForbiddenAfterPlacement"/>
        /// for how the cache is incrementally maintained.
        /// </summary>
        private Paths64 GetOrBuildForbiddenRegion(
    OrientedPart candidate,
    SheetState sheet)
        {
            // After UpdateForbiddenAfterPlacement runs on every placement, every
            // orientation has a cache entry. Empty sheet (no placements yet) means
            // no forbidden region — return empty.
            if (sheet.Placed.Count == 0)
                return new Paths64();

            if (sheet.ForbiddenByOrientation.TryGetValue(candidate.OrientationIndex, out var cached))
                return cached;

            // Should not happen if UpdateForbiddenAfterPlacement is called on every
            // placement — but stay safe and build from scratch just in case.

            // First time this orientation is queried for this sheet — build from scratch
            // against currently placed parts. After this, the cache is maintained
            // incrementally in UpdateForbiddenAfterPlacement.
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
        /// After a part is placed, update the cumulative forbidden region for EVERY
        /// candidate orientation that could possibly be queried later — including ones
        /// that haven't been queried yet on this sheet. This pays the work up front so
        /// every <see cref="GetOrBuildForbiddenRegion"/> call after this is a cache hit.
        ///
        /// Cost per placement: O(orientations × NFPs-per-pair). Constant w.r.t. placed
        /// part count — that's the win over the previous from-scratch rebuild.
        /// </summary>
        private void UpdateForbiddenAfterPlacement(
    PlacedItem newlyPlaced,
    SheetState sheet)
        {
            var sw = new System.Diagnostics.Stopwatch();
            long totalNfp = 0;
            long totalUnion = 0;
            int orientationsProcessed = 0;
            int totalNfpVerts = 0;

            foreach (var perPart in _orientationsByPart)
            {
                foreach (var candidate in perPart)
                {
                    sw.Restart();
                    var nfp = _nfpCache.Get(newlyPlaced.Orientation, candidate);
                    sw.Stop();
                    totalNfp += sw.ElapsedMilliseconds;

                    if (nfp.Count == 0) continue;

                    foreach (var nfpPoly in nfp) totalNfpVerts += nfpPoly.Count;

                    var newPaths = new Paths64();
                    foreach (var nfpPoly in nfp)
                    {
                        var translated = nfpPoly.Translate(newlyPlaced.X, newlyPlaced.Y);
                        newPaths.Add(ClipperConvert.ToPath64(translated));
                    }
                    if (newPaths.Count == 0) continue;

                    if (!sheet.ForbiddenByOrientation.TryGetValue(candidate.OrientationIndex, out var existing))
                    {
                        existing = new Paths64();
                    }

                    var merged = new Paths64(existing);
                    merged.AddRange(newPaths);

                    sw.Restart();
                    sheet.ForbiddenByOrientation[candidate.OrientationIndex]
                        = Clipper.Union(merged, FillRule.Positive);
                    sw.Stop();
                    totalUnion += sw.ElapsedMilliseconds;

                    orientationsProcessed++;
                }
            }

            DiagnosticLog?.Invoke(
                $"  UpdateForbidden: {orientationsProcessed} orientations, " +
                $"NFP {totalNfp}ms (totalVerts {totalNfpVerts}), " +
                $"Union {totalUnion}ms");
        }

        /// <summary>
        /// Linear lookup for an orientation by its index. Called rarely (only once per
        /// cache-key per placement), so a flat scan is fine.
        /// </summary>
        private OrientedPart FindOrientationByIndex(int orientationIndex)
        {
            foreach (var perPart in _orientationsByPart)
            {
                foreach (var orientation in perPart)
                {
                    if (orientation.OrientationIndex == orientationIndex)
                        return orientation;
                }
            }
            return null;
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

        /// <summary>
        /// Diagnostic helper: summarize a Paths64 region. Reports separate CCW and CW
        /// area totals so the caller can immediately see if a region is "filled" only
        /// by CW paths (which FillRule.Positive would treat as empty) or by CCW paths
        /// (which FillRule.Positive would treat as filled). Areas in model units.
        /// Also returns the bbox (min/max) of all vertices so the caller can see the
        /// region's extent in world coordinates.
        /// </summary>
        private static void DescribePaths64(
            Paths64 paths,
            out int pathCount,
            out int vertexCount,
            out double areaCcw,
            out double areaCw,
            out double minX,
            out double minY,
            out double maxX,
            out double maxY)
        {
            pathCount = paths == null ? 0 : paths.Count;
            vertexCount = 0;
            areaCcw = 0.0;
            areaCw = 0.0;
            minX = double.PositiveInfinity;
            minY = double.PositiveInfinity;
            maxX = double.NegativeInfinity;
            maxY = double.NegativeInfinity;
            if (paths == null || paths.Count == 0) return;
            double scaleSq = ClipperConvert.Scale * ClipperConvert.Scale;
            for (int i = 0; i < paths.Count; i++)
            {
                var p = paths[i];
                vertexCount += p.Count;
                double signedArea = Clipper.Area(p) / scaleSq; // signed; CCW = positive
                if (signedArea >= 0) areaCcw += signedArea;
                else areaCw += -signedArea;
                for (int j = 0; j < p.Count; j++)
                {
                    double x = p[j].X / ClipperConvert.Scale;
                    double y = p[j].Y / ClipperConvert.Scale;
                    if (x < minX) minX = x;
                    if (y < minY) minY = y;
                    if (x > maxX) maxX = x;
                    if (y > maxY) maxY = y;
                }
            }
        }

        /// <summary>
        /// Geometric point-in-polygon test against a Paths64 region using Clipper2's
        /// even-odd interior convention. Returns true if (x, y) is strictly inside or
        /// on the boundary of any path in <paramref name="paths"/>. Diagnostic-only —
        /// the engine doesn't use this anywhere; it just lets the caller verify whether
        /// the IFP corner is geometrically covered by the forbidden union, independent
        /// of whether Clipper.Difference saw fit to clip it.
        /// </summary>
        private static bool ForbiddenContainsPoint(Paths64 paths, double x, double y)
        {
            if (paths == null || paths.Count == 0) return false;
            long ix = (long)(x * ClipperConvert.Scale);
            long iy = (long)(y * ClipperConvert.Scale);
            var pt = new Point64(ix, iy);
            // Sum windings across all paths so a CCW outer + CW hole correctly cancel.
            int totalWinding = 0;
            for (int i = 0; i < paths.Count; i++)
            {
                var pip = Clipper.PointInPolygon(pt, paths[i]);
                if (pip == PointInPolygonResult.IsOn) return true;
                if (pip == PointInPolygonResult.IsInside)
                {
                    double signedArea = Clipper.Area(paths[i]);
                    totalWinding += signedArea >= 0 ? 1 : -1;
                }
            }
            return totalWinding > 0;
        }

        // ------------------------------------------------------------------
        // Per-call state
        // ------------------------------------------------------------------

        private sealed class SheetState
        {
            public List<PlacedItem> Placed { get; } = new List<PlacedItem>();

            /// <summary>
            /// Cumulative forbidden region for this sheet, keyed by candidate orientation index.
            /// As parts are placed, each candidate orientation's forbidden region is updated by
            /// unioning ONE new NFP into the running total — never recomputed from scratch.
            /// Replaces the O(N) re-union per placement attempt with O(1) work.
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