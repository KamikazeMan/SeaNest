using System;
using System.Collections.Generic;
using System.Linq;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Left-compaction post-pass. Runs AFTER placement on a valid,
    /// non-overlapping nest and slides every part as far LEFT (-X) as it can
    /// go, closing the air gaps annealing leaves between parts and
    /// accumulating the reclaimed space as a clean remnant on the RIGHT end
    /// of each sheet.
    ///
    /// Semantics:
    ///   - Per sheet. Parts never move between sheets (and therefore never
    ///     between thickness groups — the thickness-aware ReNest nests each
    ///     group into its own response before compaction runs).
    ///   - Translation ONLY, along -X. No rotation (orientation was already
    ///     optimized by annealing), no Y movement in v1.
    ///   - Spacing is respected: a candidate position is valid only if the
    ///     part's polygon stays clear of every other part's polygon INFLATED
    ///     by the request spacing (same convention as the BLF engine's
    ///     inflatedPlaced check). Compaction closes AIR, not the cut kerf
    ///     clearance.
    ///   - Physical slide, not teleport: the search advances in steps no
    ///     larger than the spacing. Because every obstacle is inflated by
    ///     spacing on both sides, its extent along X is at least 2x spacing —
    ///     a step of at most spacing cannot jump across it, so the found
    ///     position is always physically reachable by sliding.
    ///   - Deterministic: parts are processed leftmost-first with fixed
    ///     tie-breaks, the step ladder and binary refinement use fixed
    ///     constants, and there is no randomness or parallelism.
    ///
    /// The returned placements carry updated <see cref="PlacementResult.Transform"/>
    /// (the original transform composed with the slide translation) as well as
    /// the translated <see cref="PlacementResult.PlacedPolygon"/>. Ride-along
    /// geometry — holes, scribe lines, labels — is drawn through
    /// PolygonToCurve.ToCurveFromOriginal(placement.Transform), so it moves
    /// with its parent automatically; no adapter-side changes are needed.
    /// </summary>
    public static class Compactor
    {
        /// <summary>A part that moved less than this is considered settled.</summary>
        private const double MoveTolerance = 0.01;

        /// <summary>Binary-refinement resolution for the final slide distance.</summary>
        private const double RefineResolution = 0.001;

        /// <summary>Maximum left-pass iterations per sheet.</summary>
        private const int MaxPasses = 5;

        /// <summary>
        /// Compact all placements toward the left sheet edge. Returns a new
        /// list (same order as the input) with moved parts replaced by
        /// translated copies. Never introduces an overlap: every candidate
        /// position is validated against every other part on the sheet.
        /// </summary>
        /// <param name="placements">Valid, non-overlapping placements (all sheets).</param>
        /// <param name="margin">Sheet-edge inset; the left stop is x = margin.</param>
        /// <param name="spacing">Minimum part-to-part clearance to preserve.</param>
        /// <param name="diagnostic">Optional per-sheet summary sink. May be null.</param>
        public static List<PlacementResult> CompactLeft(
            IReadOnlyList<PlacementResult> placements,
            double margin,
            double spacing,
            Action<string> diagnostic)
        {
            if (placements == null) throw new ArgumentNullException(nameof(placements));

            var result = new List<PlacementResult>(placements);

            foreach (var sheetGroup in placements.GroupBy(p => p.Sheet).OrderBy(g => g.Key))
            {
                CompactSheetLeft(result, sheetGroup.Key, margin, spacing, diagnostic);
            }

            return result;
        }

        private static void CompactSheetLeft(
            List<PlacementResult> all,
            int sheetIdx,
            double margin,
            double spacing,
            Action<string> diagnostic)
        {
            // Indices into `all` of this sheet's parts.
            var idx = new List<int>();
            for (int i = 0; i < all.Count; i++)
                if (all[i].Sheet == sheetIdx) idx.Add(i);
            if (idx.Count == 0) return;

            double usedRightBefore = idx.Max(i => all[i].PlacedPolygon.BoundingBox.MaxX);

            // Inflated-obstacle cache per part on this sheet, rebuilt for a
            // part whenever it moves. Inflation by the full spacing matches
            // the engine's clearance convention (BLF inflatedPlaced / NFP
            // spacing-baked NFPs).
            var inflated = new Dictionary<int, IReadOnlyList<Polygon>>();
            var inflatedBBox = new Dictionary<int, BoundingBox2D>();
            foreach (int i in idx) RebuildInflated(all, i, spacing, inflated, inflatedBBox);

            // Step no larger than spacing so an inflated obstacle (extent
            // >= 2x spacing along X) can never be jumped. Floor keeps the
            // walk sane for spacing = 0.
            double step = spacing > 0.01 ? spacing : 0.1;

            int totalMoved = 0;
            int passesRun = 0;

            for (int pass = 1; pass <= MaxPasses; pass++)
            {
                passesRun = pass;
                double maxMove = 0.0;

                // Leftmost-first, ties by MinY then OriginalIndex —
                // deterministic, and each part compacts against neighbors
                // that have already settled to its left.
                var order = idx
                    .OrderBy(i => all[i].PlacedPolygon.BoundingBox.MinX)
                    .ThenBy(i => all[i].PlacedPolygon.BoundingBox.MinY)
                    .ThenBy(i => all[i].OriginalIndex)
                    .ToList();

                foreach (int me in order)
                {
                    double slide = FindMaxLeftSlide(
                        all, me, idx, margin, step, inflated, inflatedBBox);

                    if (slide > MoveTolerance)
                    {
                        ApplySlide(all, me, slide);
                        RebuildInflated(all, me, spacing, inflated, inflatedBBox);
                        if (slide > maxMove) maxMove = slide;
                        totalMoved++;
                    }
                }

                if (maxMove < MoveTolerance) break;
            }

            double usedRightAfter = idx.Max(i => all[i].PlacedPolygon.BoundingBox.MaxX);

            diagnostic?.Invoke(
                $"Compaction sheet {sheetIdx}: used width {usedRightBefore:F2} -> {usedRightAfter:F2} " +
                $"(remnant gained {usedRightBefore - usedRightAfter:F2}), " +
                $"{totalMoved} slide(s) over {passesRun} pass(es).");
        }

        /// <summary>
        /// Furthest valid -X slide for part <paramref name="me"/>: coarse walk
        /// in steps of <paramref name="step"/>, then binary refinement inside
        /// the final sub-step window. The walk guarantees every intermediate
        /// position was valid, so the result is physically reachable.
        /// </summary>
        private static double FindMaxLeftSlide(
            List<PlacementResult> all,
            int me,
            List<int> sheetIdxList,
            double margin,
            double step,
            Dictionary<int, IReadOnlyList<Polygon>> inflated,
            Dictionary<int, BoundingBox2D> inflatedBBox)
        {
            var poly = all[me].PlacedPolygon;
            double maxRange = poly.BoundingBox.MinX - margin;
            if (maxRange <= RefineResolution) return 0.0;

            bool Valid(double dx)
            {
                var bb = poly.BoundingBox;
                var candBBox = new BoundingBox2D(bb.MinX - dx, bb.MinY, bb.MaxX - dx, bb.MaxY);
                if (candBBox.MinX < margin - 1e-9) return false;

                Polygon candPoly = null; // translated lazily (AABB broad phase first)
                foreach (int other in sheetIdxList)
                {
                    if (other == me) continue;
                    if (!candBBox.Intersects(inflatedBBox[other])) continue;

                    if (candPoly == null) candPoly = poly.Translate(-dx, 0.0);
                    foreach (var obst in inflated[other])
                    {
                        if (OverlapChecker.Overlaps(candPoly, obst))
                            return false;
                    }
                }
                return true;
            }

            // Coarse walk.
            double lastValid = 0.0;
            double probe = step;
            while (probe <= maxRange && Valid(probe))
            {
                lastValid = probe;
                probe += step;
            }

            // Refinement window: (lastValid, upper]. The window is at most one
            // step wide (<= spacing), which an inflated obstacle cannot fit
            // inside — endpoint tests inside the window cannot tunnel.
            double upper = Math.Min(probe, maxRange);
            if (upper > lastValid && Valid(upper))
            {
                // Reached the range end (sheet margin) within the window.
                return upper;
            }

            double lo = lastValid, hi = Math.Min(probe, maxRange);
            while (hi - lo > RefineResolution)
            {
                double mid = (lo + hi) * 0.5;
                if (Valid(mid)) lo = mid;
                else hi = mid;
            }
            return lo;
        }

        private static void ApplySlide(List<PlacementResult> all, int i, double dx)
        {
            var p = all[i];
            all[i] = new PlacementResult(
                p.OriginalIndex,
                p.Sheet,
                p.Transform.Then(Transform2D.Translation(-dx, 0.0)),
                p.RotationDeg,
                p.IsMirrored,
                p.SourceBBoxMinX,
                p.SourceBBoxMaxX,
                p.PlacedPolygon.Translate(-dx, 0.0));
        }

        private static void RebuildInflated(
            List<PlacementResult> all,
            int i,
            double spacing,
            Dictionary<int, IReadOnlyList<Polygon>> inflated,
            Dictionary<int, BoundingBox2D> inflatedBBox)
        {
            var poly = all[i].PlacedPolygon;
            IReadOnlyList<Polygon> inf = spacing > 0.0
                ? PolygonInflate.Inflate(poly, spacing)
                : new[] { poly };
            if (inf.Count == 0) inf = new[] { poly };
            inflated[i] = inf;

            var bb = inf[0].BoundingBox;
            for (int k = 1; k < inf.Count; k++)
                bb = bb.Union(inf[k].BoundingBox);
            inflatedBBox[i] = bb;
        }
    }
}
