using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Overlap
{
    /// <summary>
    /// Shared int64 ↔ double conversion for Clipper2 operations.
    ///
    /// Clipper2 operates exclusively on int64 coordinates for robustness; floating-point
    /// is converted on the way in and converted back on the way out. All modules that
    /// touch Clipper2 — <see cref="PolygonInflate"/>, <see cref="OverlapChecker"/>, and
    /// the NFP engine — must use the same scale factor and the same truncation semantics
    /// or polygons will not round-trip cleanly through int64 space.
    ///
    /// Conventions (must match across all callers):
    ///   - Scale factor: <see cref="OverlapChecker.ClipperScale"/>.
    ///   - Cast direction: truncating cast <c>(long)(value * scale)</c>, NOT Math.Round.
    ///     Truncation is the existing Phase 1 convention; switching to Round here would
    ///     create a one-LSB drift between modules and silently break overlap consistency.
    ///   - Trailing-vertex policy: input <see cref="Polygon"/> never carries a duplicate
    ///     closing vertex (enforced by the <see cref="Polygon"/> constructor), and output
    ///     <see cref="Path64"/> likewise has none. Clipper's offset and Minkowski operators
    ///     produce paths without trailing duplicates, so no special handling is needed.
    /// </summary>
    public static class ClipperConvert
    {
        /// <summary>
        /// Scale factor for double → int64 conversion. Re-exported from
        /// <see cref="OverlapChecker.ClipperScale"/> so callers can reference it via this
        /// class without an extra using.
        /// </summary>
        public const double Scale = OverlapChecker.ClipperScale;

        /// <summary>
        /// Convert a <see cref="Polygon"/> to a Clipper2 <see cref="Path64"/> in scaled int64
        /// coordinates. Truncating cast — see class remarks.
        /// </summary>
        public static Path64 ToPath64(Polygon poly)
        {
            var pts = poly.Points;
            var path = new Path64(pts.Count);
            for (int i = 0; i < pts.Count; i++)
            {
                long x = (long)(pts[i].X * Scale);
                long y = (long)(pts[i].Y * Scale);
                path.Add(new Point64(x, y));
            }
            return path;
        }

        /// <summary>
        /// Convert a Clipper2 <see cref="Path64"/> back to a <see cref="Polygon"/>, rescaling
        /// to model units. Returns <c>null</c> if the path has fewer than 3 vertices —
        /// which can happen when an offset, union, or Minkowski operation collapses an input
        /// to a degenerate sliver. Callers should filter nulls.
        /// </summary>
        public static Polygon FromPath64(Path64 path)
        {
            if (path.Count < 3)
                return null;

            var points = new Point2D[path.Count];
            for (int i = 0; i < path.Count; i++)
            {
                double x = path[i].X / Scale;
                double y = path[i].Y / Scale;
                points[i] = new Point2D(x, y);
            }

            // Use the public constructor so trailing-duplicate-vertex removal and validation run.
            // Clipper2 normally returns clean paths, but this is cheap insurance.
            return new Polygon(points);
        }

        /// <summary>
        /// Convert a list of polygons to a <see cref="Paths64"/>. Convenience for callers
        /// that need to feed multiple subjects into Clipper at once (e.g. unioning all
        /// placed-part NFPs into a single forbidden region).
        /// </summary>
        public static Paths64 ToPaths64(System.Collections.Generic.IReadOnlyList<Polygon> polygons)
        {
            var paths = new Paths64(polygons.Count);
            for (int i = 0; i < polygons.Count; i++)
                paths.Add(ToPath64(polygons[i]));
            return paths;
        }

        /// <summary>
        /// Convert a <see cref="Paths64"/> result back to a list of <see cref="Polygon"/>,
        /// dropping any path that's degenerate after rescaling.
        /// </summary>
        public static System.Collections.Generic.List<Polygon> FromPaths64(Paths64 paths)
        {
            var result = new System.Collections.Generic.List<Polygon>(paths.Count);
            for (int i = 0; i < paths.Count; i++)
            {
                var poly = FromPath64(paths[i]);
                if (poly != null)
                    result.Add(poly);
            }
            return result;
        }
    }
}