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

        /// <summary>
        /// Optional diagnostic sink. When wired, every Compute call emits up to
        /// three lines: (1) input/inflate winding sanity, (2) MinkowskiDiff per-
        /// piece quad-set winding totals, (3) Union-of-quads output winding —
        /// only when CW appears in stage 2 or 3, or when input invariants fail.
        /// Used to attribute CW sub-loops in the forbidden region to ClipperOffset
        /// (Inflate) vs MinkowskiDiff vs Union.
        /// </summary>
        public static Action<string> DiagnosticLog { get; set; }

        public static IReadOnlyList<Polygon> Compute(Polygon a, Polygon b, double spacing)
        {
            if (a == null) throw new ArgumentNullException(nameof(a));
            if (b == null) throw new ArgumentNullException(nameof(b));

            // Sanity: per the FillRule contract documented above this class,
            // both inputs must be CCW. OrientedPart.Build asserts this for
            // CanonicalPolygon, so a CW input here means someone bypassed the
            // canonical pipeline. Surface it loudly via the diagnostic.
            if (DiagnosticLog != null && (!a.IsCounterClockwise || !b.IsCounterClockwise))
            {
                DiagnosticLog.Invoke(
                    $"  NFP input non-CCW! a.CCW={a.IsCounterClockwise} (area={a.Area:G6}) " +
                    $"b.CCW={b.IsCounterClockwise} (area={b.Area:G6})");
            }

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

            // Diagnostic: summarize the raw MinkowskiDiff quad set BEFORE Union.
            // Stage-2 visibility — was CW present in the quad accumulation, or
            // does it appear at the Union step?
            int quadCcwCount = 0, quadCwCount = 0;
            double quadCcwArea = 0.0, quadCwArea = 0.0;
            if (DiagnosticLog != null)
            {
                double scaleSq = ClipperConvert.Scale * ClipperConvert.Scale;
                for (int i = 0; i < allNfpPaths.Count; i++)
                {
                    double signed = Clipper.Area(allNfpPaths[i]) / scaleSq;
                    if (signed >= 0) { quadCcwCount++; quadCcwArea += signed; }
                    else { quadCwCount++; quadCwArea += -signed; }
                }
            }

            // Union all NFPs from each A piece.
            Paths64 unioned = Clipper.Union(allNfpPaths, FillRule.Positive);

            // Pre-fix winding counts (diagnostic only — measure what Union actually
            // produced before we normalize it).
            int preFixCcwCount = 0, preFixCwCount = 0;
            double preFixCcwArea = 0.0, preFixCwArea = 0.0;
            if (DiagnosticLog != null && unioned != null)
            {
                double scaleSq = ClipperConvert.Scale * ClipperConvert.Scale;
                for (int i = 0; i < unioned.Count; i++)
                {
                    double signed = Clipper.Area(unioned[i]) / scaleSq;
                    if (signed >= 0) { preFixCcwCount++; preFixCcwArea += signed; }
                    else { preFixCwCount++; preFixCwArea += -signed; }
                }
            }

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
            // uniformly CCW. The contract documented at this class header — "NFP
            // outputs are CCW; downstream FillRule.Positive interprets them as
            // filled regions" — is now actually enforced by code, not just hoped
            // for.
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
                        unioned[i].Reverse();
                }
            }

            // Diagnostic: emit pre-fix and post-fix winding side-by-side so the
            // next run shows the fix actually flipped any CW that arrived. Same
            // filter as before — only emit when CW was present somewhere
            // pre-fix or in the raw Minkowski quads.
            if (DiagnosticLog != null && (quadCwCount > 0 || preFixCwCount > 0))
            {
                int postFixCcwCount = 0, postFixCwCount = 0;
                double postFixCcwArea = 0.0, postFixCwArea = 0.0;
                if (unioned != null)
                {
                    double scaleSq = ClipperConvert.Scale * ClipperConvert.Scale;
                    for (int i = 0; i < unioned.Count; i++)
                    {
                        double signed = Clipper.Area(unioned[i]) / scaleSq;
                        if (signed >= 0) { postFixCcwCount++; postFixCcwArea += signed; }
                        else { postFixCwCount++; postFixCwArea += -signed; }
                    }
                }

                DiagnosticLog.Invoke(
                    $"  NFP a(verts={a.Count}, area={a.Area:G6}) b(verts={b.Count}, area={b.Area:G6}) spacing={spacing:G6}: " +
                    $"quads {allNfpPaths.Count}p ({quadCcwCount}CCW area={quadCcwArea:G6} | {quadCwCount}CW area={quadCwArea:G6}) -> " +
                    $"union {(unioned == null ? 0 : unioned.Count)}p pre-fix ({preFixCcwCount}CCW area={preFixCcwArea:G6} | {preFixCwCount}CW area={preFixCwArea:G6}) " +
                    $"post-fix ({postFixCcwCount}CCW area={postFixCcwArea:G6} | {postFixCwCount}CW area={postFixCwArea:G6})");
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