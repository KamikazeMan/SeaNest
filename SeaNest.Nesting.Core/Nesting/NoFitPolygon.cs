using System;
using System.Collections.Generic;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    public static class NoFitPolygon
    {
        private static string LogPath => System.IO.Path.Combine(
            System.Environment.GetFolderPath(System.Environment.SpecialFolder.Desktop),
            "seanest_nfp.log");

        private static void Log(string message)
        {
            try { System.IO.File.AppendAllText(LogPath, message + "\r\n"); }
            catch { }
        }

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

            Log($"=== NFP.Compute entered: a={a.Count}v b={b.Count}v inflatedA.Count={inflatedA.Count} ===");

            var sw = new System.Diagnostics.Stopwatch();
            foreach (var aPiece in inflatedA)
            {
                Path64 aPath = ClipperConvert.ToPath64(aPiece);

                sw.Restart();
                Paths64 nfpPiece = Clipper.MinkowskiDiff(bPath, aPath, isClosed: true);
                sw.Stop();

                int rawVerts = 0;
                if (nfpPiece != null)
                    foreach (var p in nfpPiece) rawVerts += p.Count;

                Log($"  MinkowskiDiff: a={aPath.Count}v b={bPath.Count}v -> {(nfpPiece?.Count ?? 0)} paths, {rawVerts} verts in {sw.ElapsedMilliseconds}ms");

                if (nfpPiece == null || nfpPiece.Count == 0) continue;

                allNfpPaths.AddRange(nfpPiece);
            }

            if (allNfpPaths.Count == 0)
                return Array.Empty<Polygon>();

            // Union all NFPs from each A piece.
            sw.Restart();
            Paths64 unioned = Clipper.Union(allNfpPaths, FillRule.Positive);
            sw.Stop();

            int unionVerts = 0;
            if (unioned != null)
                foreach (var p in unioned) unionVerts += p.Count;

            Log($"  Union: {unionVerts} verts in {sw.ElapsedMilliseconds}ms");

            if (unioned == null || unioned.Count == 0)
                return Array.Empty<Polygon>();

            // Convert to Polygon list, then simplify each polygon using our own
            // collinear-removal algorithm. Same tolerance as BrepFlattener's input
            // simplification — preserves placement geometry within CAM precision.
            var polygons = ClipperConvert.FromPaths64(unioned);

            sw.Restart();
            var simplified = new List<Polygon>(polygons.Count);
            int beforeVerts = 0, afterVerts = 0;
            foreach (var poly in polygons)
            {
                beforeVerts += poly.Count;
                var s = poly.Simplify(NfpSimplifyTolerance);
                afterVerts += s.Count;
                simplified.Add(s);
            }
            sw.Stop();

            Log($"  Simplify: {beforeVerts} -> {afterVerts} verts in {sw.ElapsedMilliseconds}ms");

            return simplified;
        }
    }
}