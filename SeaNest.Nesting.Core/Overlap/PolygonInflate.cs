using System;
using System.Collections.Generic;
using System.Text;
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
        /// Optional diagnostic sink. When wired, every Inflate call where the
        /// output contains a CW path (unexpected) emits a one-line summary of
        /// the input polygon's signed area + winding and each output polygon's
        /// signed area + winding. Used to attribute spurious CW sub-loops in
        /// the forbidden region to ClipperOffset vs Minkowski/Union.
        /// </summary>
        public static Action<string> DiagnosticLog { get; set; }

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

            var result = ClipperConvert.FromPaths64(solution);

            // Diagnostic: report when output contains a CW path. CCW input + CCW
            // output is the expected fast path and is silenced.
            if (DiagnosticLog != null)
            {
                int cwCount = 0;
                for (int i = 0; i < result.Count; i++)
                    if (!result[i].IsCounterClockwise) cwCount++;
                if (cwCount > 0)
                {
                    var sb = new StringBuilder();
                    sb.Append($"  Inflate delta={delta:G6} input(verts={polygon.Count}, area={polygon.Area:G6}, CCW={polygon.IsCounterClockwise}) -> {result.Count}p");
                    for (int i = 0; i < result.Count; i++)
                    {
                        sb.Append(i == 0 ? " [" : "; ");
                        sb.Append($"verts={result[i].Count}, area={result[i].Area:G6}, CCW={result[i].IsCounterClockwise}");
                    }
                    if (result.Count > 0) sb.Append("]");
                    DiagnosticLog.Invoke(sb.ToString());
                }
            }

            return result;
        }
    }
}