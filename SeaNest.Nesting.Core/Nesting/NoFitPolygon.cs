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
        /// Total number of CW paths reversed to CCW across all <see cref="Compute"/>
        /// calls in the current process. Diagnostic only — used by the engine to
        /// surface a confidence summary line per UpdateForbidden when corrections
        /// occur. Reset by callers via <see cref="ResetCwCorrectionsCounter"/> at
        /// the start of a nest run.
        /// </summary>
        public static int CwCorrectionsTotal { get; private set; }

        /// <summary>Reset the CW-correction counter to zero.</summary>
        public static void ResetCwCorrectionsCounter() => CwCorrectionsTotal = 0;

        public static IReadOnlyList<Polygon> Compute(Polygon a, Polygon b, double spacing)
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
            // CW-to-CCW normalization at the Paths64 layer.
            //
            // Clipper2 v2.0.0's MinkowskiDiff emits CW sub-paths for certain shape
            // pairs even when both inputs are CCW (confirmed by the NFP/Inflate
            // attribution diagnostic — CW area appears in the raw quad set, not
            // born inside Clipper.Union). FillRule.Positive then carries those CW
            // paths through Union faithfully because they have nowhere to merge.
            // Downstream NfpPlacementEngine treats them as holes in the cumulative
            // forbidden region under FillRule.Positive, which silently re-permits
            // exactly the placement positions occupied by already-placed parts —
            // root cause of the engine-vs-verifier overlap bug.
            //
            // Fix: walk the Union output and reverse any CW path so the result is
            // uniformly CCW. The contract documented at this class header is now
            // enforced by code, not just hoped for.
            //
            // KNOWN LIMITATION (convex-only safety): For genuinely concave A or B,
            // some CW sub-paths in MinkowskiDiff output legitimately encode holes
            // in the NFP — regions of (B+t) translation that DON'T cause overlap
            // despite being spatially inside the outer NFP boundary. Blanket CW-
            // to-CCW reversal would erase those holes, leaving over-constrained
            // forbidden regions (no false placements, but possibly missed valid
            // placements in tight nests). The current SeaNest test set is convex;
            // when real concave boat parts come through, this fix needs to
            // distinguish "spurious CW noise from MinkowskiDiff" from "legitimate
            // CW hole encoding a concavity". Likely path: keep CW paths whose
            // |area| exceeds a noise threshold AND whose vertices lie strictly
            // inside the outer CCW envelope (real holes); reverse the rest.
            // ------------------------------------------------------------------
            if (unioned != null)
            {
                for (int i = 0; i < unioned.Count; i++)
                {
                    if (Clipper.Area(unioned[i]) < 0)
                    {
                        unioned[i].Reverse();
                        CwCorrectionsTotal++;
                    }
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
    }
}
