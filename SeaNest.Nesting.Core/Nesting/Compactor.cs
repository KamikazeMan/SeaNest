using System;
using System.Collections.Generic;
using System.Linq;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Left+down compaction post-pass, iterated to convergence. Runs AFTER
    /// placement on a valid, non-overlapping nest and alternates LEFT passes
    /// (every part slides as far -X as valid) with DOWN passes (as far -Y as
    /// valid) until a full left+down pair moves nothing beyond tolerance.
    ///
    /// Why the alternation matters (v1 was pure-left): a part with open space
    /// far to its left but a blocker in between cannot get there by sliding
    /// left alone. Dropping DOWN into a vertical gap lets it clear the
    /// blocker, and the next LEFT pass carries it further. Iterating the pair
    /// to convergence lets right-side parts progressively migrate into the
    /// leftmost space they can legally occupy instead of settling against
    /// their immediate neighbor.
    ///
    /// Semantics:
    ///   - Per sheet. Parts never move between sheets (and therefore never
    ///     between thickness groups).
    ///   - Translation ONLY (-X and -Y). No rotation — the anneal owns
    ///     orientation.
    ///   - Primary bias LEFT: the pair runs left-then-down, so horizontal
    ///     packing leads and the remnant accumulates on the RIGHT end of the
    ///     sheet. DOWN exists to close vertical gaps and unlock further left
    ///     travel.
    ///   - Spacing is respected: a candidate position is valid only if the
    ///     part's polygon stays clear of every other part's polygon INFLATED
    ///     by the request spacing (the engine's own clearance convention).
    ///     Compaction closes AIR, not the cut kerf clearance.
    ///   - Physical slide, not teleport: the search advances in steps no
    ///     larger than the spacing. Every obstacle is inflated by spacing on
    ///     both sides, so its extent along the slide axis is at least
    ///     2x spacing — a step of at most spacing cannot jump across it, and
    ///     the found position is always reachable by sliding.
    ///   - Deterministic: fixed per-pass ordering (left pass leftmost-first,
    ///     down pass bottommost-first, fixed tie-breaks), fixed step ladder
    ///     and refinement, fixed iteration cap, no randomness, no
    ///     parallelism. Same input -> byte-identical output.
    ///
    /// The returned placements carry updated <see cref="PlacementResult.Transform"/>
    /// (original transform composed with the accumulated slide translation)
    /// and the translated <see cref="PlacementResult.PlacedPolygon"/>.
    /// Ride-along geometry — holes, scribe lines, labels — is drawn through
    /// PolygonToCurve.ToCurveFromOriginal(placement.Transform), so it moves
    /// with its parent automatically.
    /// </summary>
    public static class Compactor
    {
        /// <summary>A slide smaller than this is considered no movement.</summary>
        private const double MoveTolerance = 0.01;

        /// <summary>Binary-refinement resolution for the final slide distance.</summary>
        private const double RefineResolution = 0.001;

        /// <summary>Maximum left+down pair iterations per sheet.</summary>
        private const int MaxPairIterations = 12;

        /// <summary>
        /// Compact all placements left and down, iterated to convergence.
        /// Returns a new list (same order as the input) with moved parts
        /// replaced by translated copies. Never introduces an overlap.
        /// </summary>
        /// <param name="placements">Valid, non-overlapping placements (all sheets).</param>
        /// <param name="margin">Sheet-edge inset; left stop x = margin, bottom stop y = margin.</param>
        /// <param name="spacing">Minimum part-to-part clearance to preserve.</param>
        /// <param name="diagnostic">Optional per-sheet summary sink. May be null.</param>
        public static List<PlacementResult> Compact(
            IReadOnlyList<PlacementResult> placements,
            double margin,
            double spacing,
            Action<string> diagnostic)
        {
            if (placements == null) throw new ArgumentNullException(nameof(placements));

            var result = new List<PlacementResult>(placements);

            foreach (var sheetGroup in placements.GroupBy(p => p.Sheet).OrderBy(g => g.Key))
            {
                CompactSheet(result, sheetGroup.Key, margin, spacing, diagnostic);
            }

            return result;
        }

        private static void CompactSheet(
            List<PlacementResult> all,
            int sheetIdx,
            double margin,
            double spacing,
            Action<string> diagnostic)
        {
            var idx = new List<int>();
            for (int i = 0; i < all.Count; i++)
                if (all[i].Sheet == sheetIdx) idx.Add(i);
            if (idx.Count == 0) return;

            double usedRightBefore = idx.Max(i => all[i].PlacedPolygon.BoundingBox.MaxX);

            var inflated = new Dictionary<int, IReadOnlyList<Polygon>>();
            var inflatedBBox = new Dictionary<int, BoundingBox2D>();
            foreach (int i in idx) RebuildInflated(all, i, spacing, inflated, inflatedBBox);

            // Step no larger than spacing so an inflated obstacle (extent
            // >= 2x spacing along either axis) can never be jumped. Floor
            // keeps the walk sane for spacing = 0.
            double step = spacing > 0.01 ? spacing : 0.1;

            int totalSlides = 0;
            int pairsRun = 0;

            for (int pair = 1; pair <= MaxPairIterations; pair++)
            {
                pairsRun = pair;

                // LEFT pass: leftmost-first (ties: MinY, OriginalIndex).
                double leftMax = RunDirectionalPass(
                    all, idx, margin, step, alongX: true,
                    inflated, inflatedBBox, spacing, ref totalSlides);

                // DOWN pass: bottommost-first (ties: MinX, OriginalIndex).
                double downMax = RunDirectionalPass(
                    all, idx, margin, step, alongX: false,
                    inflated, inflatedBBox, spacing, ref totalSlides);

                if (Math.Max(leftMax, downMax) < MoveTolerance)
                    break;
            }

            double usedRightAfter = idx.Max(i => all[i].PlacedPolygon.BoundingBox.MaxX);

            diagnostic?.Invoke(
                $"Compaction sheet {sheetIdx}: used width {usedRightBefore:F2} -> {usedRightAfter:F2} " +
                $"(remnant gained {usedRightBefore - usedRightAfter:F2}), " +
                $"{totalSlides} slide(s) over {pairsRun} left+down iteration(s).");
        }

        /// <summary>
        /// One directional pass over every part on the sheet. Returns the
        /// largest single slide applied (0 when nothing moved).
        /// </summary>
        private static double RunDirectionalPass(
            List<PlacementResult> all,
            List<int> idx,
            double margin,
            double step,
            bool alongX,
            Dictionary<int, IReadOnlyList<Polygon>> inflated,
            Dictionary<int, BoundingBox2D> inflatedBBox,
            double spacing,
            ref int totalSlides)
        {
            double maxMove = 0.0;

            // Left pass: leftmost-first so each part settles against already-
            // compacted neighbors. Down pass: bottommost-first, same logic
            // rotated 90 degrees.
            var order = alongX
                ? idx.OrderBy(i => all[i].PlacedPolygon.BoundingBox.MinX)
                     .ThenBy(i => all[i].PlacedPolygon.BoundingBox.MinY)
                     .ThenBy(i => all[i].OriginalIndex)
                     .ToList()
                : idx.OrderBy(i => all[i].PlacedPolygon.BoundingBox.MinY)
                     .ThenBy(i => all[i].PlacedPolygon.BoundingBox.MinX)
                     .ThenBy(i => all[i].OriginalIndex)
                     .ToList();

            foreach (int me in order)
            {
                double slide = FindMaxSlide(
                    all, me, idx, margin, step, alongX, inflated, inflatedBBox);

                if (slide > MoveTolerance)
                {
                    ApplySlide(all, me, alongX ? slide : 0.0, alongX ? 0.0 : slide);
                    RebuildInflated(all, me, spacing, inflated, inflatedBBox);
                    if (slide > maxMove) maxMove = slide;
                    totalSlides++;
                }
            }

            return maxMove;
        }

        /// <summary>
        /// Furthest valid slide for part <paramref name="me"/> along -X
        /// (<paramref name="alongX"/> = true) or -Y (false): coarse walk in
        /// steps of <paramref name="step"/>, then binary refinement inside the
        /// final sub-step window. The walk guarantees every intermediate
        /// position was valid, so the result is physically reachable.
        /// </summary>
        private static double FindMaxSlide(
            List<PlacementResult> all,
            int me,
            List<int> sheetIdxList,
            double margin,
            double step,
            bool alongX,
            Dictionary<int, IReadOnlyList<Polygon>> inflated,
            Dictionary<int, BoundingBox2D> inflatedBBox)
        {
            var poly = all[me].PlacedPolygon;
            var bb = poly.BoundingBox;
            double maxRange = (alongX ? bb.MinX : bb.MinY) - margin;
            if (maxRange <= RefineResolution) return 0.0;

            bool Valid(double d)
            {
                var candBBox = alongX
                    ? new BoundingBox2D(bb.MinX - d, bb.MinY, bb.MaxX - d, bb.MaxY)
                    : new BoundingBox2D(bb.MinX, bb.MinY - d, bb.MaxX, bb.MaxY - d);
                if ((alongX ? candBBox.MinX : candBBox.MinY) < margin - 1e-9) return false;

                Polygon candPoly = null; // translated lazily (AABB broad phase first)
                foreach (int other in sheetIdxList)
                {
                    if (other == me) continue;
                    if (!candBBox.Intersects(inflatedBBox[other])) continue;

                    if (candPoly == null)
                        candPoly = alongX ? poly.Translate(-d, 0.0) : poly.Translate(0.0, -d);
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

            // Refinement window is at most one step wide (<= spacing), which
            // an inflated obstacle cannot fit inside — endpoint tests inside
            // the window cannot tunnel.
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

        private static void ApplySlide(List<PlacementResult> all, int i, double dx, double dy)
        {
            var p = all[i];
            all[i] = new PlacementResult(
                p.OriginalIndex,
                p.Sheet,
                p.Transform.Then(Transform2D.Translation(-dx, -dy)),
                p.RotationDeg,
                p.IsMirrored,
                p.SourceBBoxMinX,
                p.SourceBBoxMaxX,
                p.PlacedPolygon.Translate(-dx, -dy));
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
