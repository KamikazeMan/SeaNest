using System.Collections.Generic;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Overlap
{
    /// <summary>
    /// Polygon inflation (offsetting) using Clipper2.
    ///
    /// Used for spacing enforcement and sheet-margin insets during nesting:
    /// positive delta grows the polygon outward, negative delta shrinks it inward.
    /// Shrinking a thin polygon past its own width produces zero output paths;
    /// inflating a polygon with narrow necks can produce multiple disjoint output paths.
    /// Both cases are represented naturally by returning a list.
    ///
    /// Uses <see cref="ClipperConvert"/> for int64 conversion so results are numerically
    /// consistent with overlap detection, NFP, and any other Clipper2-based module.
    /// </summary>
    public static class PolygonInflate
    {
        /// <summary>
        /// Miter limit for corner handling, in multiples of the offset delta.
        /// 2.0 is Clipper2's default and preserves sharp corners on typical sheet-metal geometry
        /// without producing pathological spikes on very acute angles.
        /// </summary>
        private const double MiterLimit = 2.0;

        /// <summary>
        /// Inflate (offset) a polygon by the given delta, in model units.
        ///
        /// Positive delta grows the polygon outward (e.g. for spacing buffers around placed parts).
        /// Negative delta shrinks the polygon inward (e.g. for sheet-margin insets).
        ///
        /// Returns a list of resulting polygons:
        ///   - Usually one polygon.
        ///   - Zero polygons if a negative delta collapses the input entirely.
        ///   - Multiple polygons if a positive delta merges parts of the input or if a negative
        ///     delta splits a narrow-necked input into disjoint pieces.
        /// </summary>
        public static IReadOnlyList<Polygon> Inflate(Polygon polygon, double delta)
        {
            var subject = new Paths64 { ClipperConvert.ToPath64(polygon) };

            double scaledDelta = delta * ClipperConvert.Scale;

            var offsetter = new ClipperOffset(MiterLimit);
            offsetter.AddPaths(subject, JoinType.Miter, EndType.Polygon);

            var solution = new Paths64();
            offsetter.Execute(scaledDelta, solution);

            return ClipperConvert.FromPaths64(solution);
        }
    }
}