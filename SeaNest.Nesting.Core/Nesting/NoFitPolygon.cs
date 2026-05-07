using System;
using System.Collections.Generic;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Compute the No-Fit Polygon of two oriented parts via Clipper2's MinkowskiDiff.
    ///
    /// FillRule contract:
    ///   This class and the rest of the NFP pipeline (NfpPlacementEngine) use
    ///   <see cref="FillRule.Positive"/> because all inputs are guaranteed CCW:
    ///     - <see cref="OrientedPart.Build"/> forces ToCounterClockwise on every
    ///       CanonicalPolygon and asserts the result.
    ///     - PolygonInflate.Inflate of a CCW polygon yields a CCW outer loop.
    ///     - Clipper2's MinkowskiDiff sometimes emits CW sub-loops even for CCW
    ///       inputs (confirmed empirically — the CW originates in the raw quad
    ///       set before any Union step). Compute reverses those at the Paths64
    ///       layer before returning, so the FillRule.Positive contract is
    ///       enforced by code, not by hope. See the comment block on the
    ///       reversal loop for the convex-only safety limitation.
    ///
    ///   Do NOT change to FillRule.NonZero without auditing how MinkowskiDiff output
    ///   self-touching paths interact with hole encoding — NonZero would treat CW
    ///   sub-loops as additional positive-fill regions, potentially over-filling the
    ///   forbidden region and rejecting valid placements. The OverlapChecker
    ///   deliberately uses NonZero for the opposite reason (mixed-winding inputs);
    ///   the asymmetry is intentional and documented at OverlapChecker class level.
    /// </summary>
    public static class NoFitPolygon
    {
        /// <summary>
        /// Tolerance for simplifying NFP output polygons, in model units.
        /// NFP outputs from MinkowskiDiff have dense vertex runs along edges — each
        /// pair of input vertices contributes a quad cell to the internal accumulation.
        /// Collinear-only simplification at this tolerance preserves placement
        /// correctness within CAM precision and drastically reduces downstream
        /// Union complexity.
        ///
        /// 0.01" is well below any sheet-metal cut tolerance and matches the input
        /// simplification tolerance set in BrepFlattener.
        /// </summary>
        private const double NfpSimplifyTolerance = 0.01;

        /// <summary>
        /// Sliver classification thresholds for the concave-aware CW filter.
        /// A CW path is "sliver" (numerical noise from MinkowskiDiff) when its
        /// |area| is below max(SliverFractionOfOuter × outerArea, SliverAbsoluteFloor).
        /// Belt-and-suspenders: fraction handles scale, absolute floor handles
        /// edge cases where outer area itself is small.
        ///   - 0.001 (0.1%) cleanly separates observed sliver-noise (~0.03% of
        ///     outer in convex test data) from real rat-hole encodings (~1–5%
        ///     of outer on boat-part NFPs).
        ///   - 0.01 sq model units = 0.1×0.1 unit, well below any CAM tolerance.
        /// </summary>
        private const double SliverFractionOfOuter = 0.001;
        private const double SliverAbsoluteFloor = 0.01;

        /// <summary>
        /// Number of CW paths reversed to CCW across all <see cref="Compute"/>
        /// calls in the current nest run. Reset per run via <see cref="ResetCounters"/>.
        /// Includes both:
        ///   - Convex-input blanket reversals (every CW from a convex × convex
        ///     pair is necessarily a numerical artifact; whole-output blanket
        ///     reverse without per-path classification).
        ///   - Concave-input by-area sliver reversals (CW paths whose |area|
        ///     falls below the noise threshold of the area+PIP classifier).
        /// Both buckets count toward the same total because they share the
        /// "reverse a CW that isn't a real hole" semantics.
        /// </summary>
        public static int SliverReversalsTotal { get; private set; }

        /// <summary>Number of CW paths kept as-is because they encode legitimate holes.</summary>
        public static int HolesPreservedTotal { get; private set; }

        /// <summary>
        /// Number of CW paths that had at least one vertex outside the outer envelope
        /// — pathological for well-formed Minkowski outputs. Reversed defensively;
        /// per-anomaly detail emitted to <see cref="AnomalyLog"/>.
        /// </summary>
        public static int AnomalyReversalsTotal { get; private set; }

        /// <summary>
        /// Optional per-anomaly diagnostic sink. Wired by NestingEngine.NestNFP
        /// for the duration of a nest run and cleared on exit. One line per
        /// anomaly so the volume stays readable; if a real-parts run produces
        /// many, we have the (src-orient, cand-orient) pairs to investigate.
        /// </summary>
        public static Action<string> AnomalyLog { get; set; }

        /// <summary>Reset all counters. Call once at the start of a nest run.</summary>
        public static void ResetCounters()
        {
            SliverReversalsTotal = 0;
            HolesPreservedTotal = 0;
            AnomalyReversalsTotal = 0;
        }

        /// <summary>
        /// Compute the no-fit polygon of <paramref name="a"/> (placed) and
        /// <paramref name="b"/> (candidate) at the given spacing. Optional
        /// orientation indices are used only for diagnostic attribution when
        /// the concave-aware CW filter encounters an anomaly; pass -1 (default)
        /// for non-engine callers who don't have orientation context.
        /// </summary>
        public static IReadOnlyList<Polygon> Compute(
            Polygon a, Polygon b, double spacing,
            int srcOrientationIndex = -1,
            int candOrientationIndex = -1)
        {
            if (a == null) throw new ArgumentNullException(nameof(a));
            if (b == null) throw new ArgumentNullException(nameof(b));

            IReadOnlyList<Polygon> inflatedA = spacing > 0.0
                ? PolygonInflate.Inflate(a, spacing)
                : new[] { a };

            if (inflatedA.Count == 0)
                return Array.Empty<Polygon>();

            Path64 bPath = ClipperConvert.ToPath64(b);

            var allNfpPaths = new Paths64();

            foreach (var aPiece in inflatedA)
            {
                Path64 aPath = ClipperConvert.ToPath64(aPiece);

                Paths64 nfpPiece = Clipper.MinkowskiDiff(bPath, aPath, isClosed: true);

                if (nfpPiece == null || nfpPiece.Count == 0) continue;

                allNfpPaths.AddRange(nfpPiece);
            }

            if (allNfpPaths.Count == 0)
                return Array.Empty<Polygon>();

            // Union all NFPs from each A piece.
            Paths64 unioned = Clipper.Union(allNfpPaths, FillRule.Positive);

            // ------------------------------------------------------------------
            // Concave-aware CW filter at the Paths64 layer.
            //
            // Background: Clipper2 v2.0.0's MinkowskiDiff emits CW sub-paths for
            // certain shape pairs even when both inputs are CCW (the CW area
            // appears in the raw quad set, not born inside Clipper.Union).
            // FillRule.Positive carries those through Union faithfully because
            // they have nowhere to merge. Downstream NfpPlacementEngine then
            // treats CW sub-loops as holes in the cumulative forbidden region,
            // which silently re-permits exactly the placement positions occupied
            // by already-placed parts.
            //
            // Filter classifies each CW path into one of three buckets:
            //   1. SLIVER — |area| below noise threshold (numerical artifact of
            //      MinkowskiDiff). Reverse to CCW so it merges into the outer
            //      forbidden region. Increments SliverReversalsTotal.
            //   2. LEGITIMATE HOLE — all vertices inside the outer envelope.
            //      Genuine concave-geometry hole encoding; KEEP as CW so
            //      FillRule.Positive subtracts it from the forbidden region
            //      correctly. Increments HolesPreservedTotal.
            //   3. ANOMALY — at least one vertex outside the outer envelope.
            //      Pathological for well-formed Minkowski outputs. Reverse
            //      defensively (over-constrains forbidden region, never
            //      under-constrains), increment AnomalyReversalsTotal, and
            //      emit one diagnostic line per anomaly via AnomalyLog.
            //
            // Outer envelope = the CCW path with the largest |area| in the
            // unioned set. Multi-region NFPs (multiple disjoint CCW outers)
            // would mis-classify CW paths in non-largest regions as anomalies;
            // accepted limitation, deferred until observed.
            //
            // PIP test is per-vertex via Clipper.PointInPolygon (IsInside or
            // IsOn count as inside). Misses pathological edge-crossings where
            // every vertex is inside the envelope but an edge exits and re-
            // enters; not produced by typical Minkowski outputs. Upgrade to
            // Clipper.Intersect-based containment if observed.
            // ------------------------------------------------------------------
            // Convexity gate: NFP of two convex polygons is mathematically
            // convex (single CCW outer boundary, no holes). Any CW sub-loop in
            // the Union output is therefore necessarily a numerical artifact
            // — blanket reverse is correct AND avoids the misclassification
            // trap where the area+PIP filter would treat a substantial CW
            // artifact (e.g. ~3% of outer area) as a "real hole" because the
            // spatial criteria match: artifact CW paths from convex×convex
            // MinkowskiDiff also have all vertices inside the outer envelope.
            //
            // Test on inflatedA, not on raw a: PolygonInflate's positive-delta
            // offset preserves convexity in theory, but verifying on the
            // actual MinkowskiDiff input keeps the contract local. The
            // Count==1 check rules out the (impossible-for-convex-input)
            // ClipperOffset-splits-into-pieces case; if it ever fires we
            // fall through to the classifier.
            if (unioned != null && unioned.Count > 0)
            {
                bool inflatedAConvex = inflatedA.Count == 1 && inflatedA[0].IsConvex;
                bool bothConvex = inflatedAConvex && b.IsConvex;

                if (bothConvex)
                {
                    ApplyConvexBlanketReverse(unioned);
                }
                else
                {
                    ApplyConcaveAwareCwFilter(unioned, srcOrientationIndex, candOrientationIndex);
                }
            }

            if (unioned == null || unioned.Count == 0)
                return Array.Empty<Polygon>();

            // Convert to Polygon list, then simplify each polygon using our own
            // collinear-removal algorithm. Same tolerance as BrepFlattener's input
            // simplification — preserves placement geometry within CAM precision.
            var polygons = ClipperConvert.FromPaths64(unioned);

            var simplified = new List<Polygon>(polygons.Count);
            foreach (var poly in polygons)
                simplified.Add(poly.Simplify(NfpSimplifyTolerance));

            return simplified;
        }

        /// <summary>
        /// Walk <paramref name="unioned"/> in place: identify the outer envelope
        /// (largest CCW by |area|), then classify each CW path as sliver, hole,
        /// or anomaly per the rules described in <see cref="Compute"/>'s comment
        /// block. Reverses sliver and anomaly paths to CCW; preserves hole paths
        /// as CW. Updates the static counters and emits per-anomaly diagnostic
        /// lines via <see cref="AnomalyLog"/>.
        /// </summary>
        private static void ApplyConcaveAwareCwFilter(
            Paths64 unioned, int srcOrientationIndex, int candOrientationIndex)
        {
            double scaleSq = ClipperConvert.Scale * ClipperConvert.Scale;

            // Pass 1: find outer envelope (largest CCW by absolute area).
            int outerIdx = -1;
            double outerArea = 0.0;
            for (int i = 0; i < unioned.Count; i++)
            {
                double signed = Clipper.Area(unioned[i]) / scaleSq;
                if (signed > outerArea)
                {
                    outerArea = signed;
                    outerIdx = i;
                }
            }

            // Degenerate: no CCW path at all. Fall back to legacy blanket reverse
            // — pathological enough that we can't trust the envelope test, but
            // the FillRule.Positive contract still requires CCW outputs.
            if (outerIdx < 0)
            {
                for (int i = 0; i < unioned.Count; i++)
                {
                    if (Clipper.Area(unioned[i]) < 0)
                    {
                        unioned[i].Reverse();
                        SliverReversalsTotal++;
                    }
                }
                return;
            }

            var outerPath = unioned[outerIdx];
            double noiseThreshold = System.Math.Max(SliverFractionOfOuter * outerArea, SliverAbsoluteFloor);

            // Pass 2: classify each non-outer path.
            for (int i = 0; i < unioned.Count; i++)
            {
                if (i == outerIdx) continue;

                double signed = Clipper.Area(unioned[i]) / scaleSq;
                if (signed >= 0) continue; // CCW non-outer (e.g. island in a hole) — keep as-is.

                double absArea = -signed;

                if (absArea < noiseThreshold)
                {
                    unioned[i].Reverse();
                    SliverReversalsTotal++;
                    continue;
                }

                // Legitimate-hole vs anomaly: short-circuit on first IsOutside.
                bool allInside = true;
                var cw = unioned[i];
                for (int v = 0; v < cw.Count; v++)
                {
                    if (Clipper.PointInPolygon(cw[v], outerPath) == PointInPolygonResult.IsOutside)
                    {
                        allInside = false;
                        break;
                    }
                }

                if (allInside)
                {
                    HolesPreservedTotal++;
                    // Keep CW; FillRule.Positive will subtract it from the union downstream.
                }
                else
                {
                    AnomalyReversalsTotal++;
                    AnomalyLog?.Invoke(
                        $"  NFP anomaly: src-orient={srcOrientationIndex} cand-orient={candOrientationIndex}, " +
                        $"CW path (verts={cw.Count}, area={absArea:G6}) had vertex outside outer envelope; " +
                        $"reversed defensively.");
                    unioned[i].Reverse();
                }
            }
        }

        /// <summary>
        /// Walk <paramref name="unioned"/> in place and reverse every CW path
        /// to CCW. Used when both NFP inputs are convex: their Minkowski
        /// difference is mathematically convex, so any CW sub-loop in the
        /// Union output is necessarily a numerical artifact of the per-quad
        /// integer arithmetic — never a real hole. No threshold or PIP test
        /// needed; that machinery is for the concave case where the NFP
        /// can legitimately have holes.
        /// </summary>
        private static void ApplyConvexBlanketReverse(Paths64 unioned)
        {
            for (int i = 0; i < unioned.Count; i++)
            {
                if (Clipper.Area(unioned[i]) < 0)
                {
                    unioned[i].Reverse();
                    SliverReversalsTotal++;
                }
            }
        }
    }
}
