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
    ///     - Clipper2's MinkowskiDiff outputs are conventionally CCW for the outer
    ///       boundary; CW sub-loops, when present, encode holes — exactly what
    ///       FillRule.Positive interprets correctly.
    ///   Positive lets the union/difference machinery treat CW sub-paths as holes
    ///   (subtracted from the outer fill) without an explicit hole-detection pass.
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