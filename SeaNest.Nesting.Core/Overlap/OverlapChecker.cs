using System.Collections.Generic;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Overlap
{
    /// <summary>
    /// Polygon-vs-polygon overlap detection using Clipper2 intersection.
    ///
    /// Single source of truth for "do these two polygons overlap?" — called by both the
    /// nesting engine (during candidate placement) and the final verifier (before writing
    /// to the Rhino document). Both callers share the same answer, so the engine cannot
    /// place parts that the verifier will then reject.
    ///
    /// Internally scales doubles into Clipper's long coordinate space at <see cref="ClipperScale"/>
    /// precision (0.0001 units), which is far finer than any real sheet-metal tolerance.
    ///
    /// FillRule contract:
    ///   This class uses <see cref="FillRule.NonZero"/> because its inputs are
    ///   user-provided polygons of arbitrary winding:
    ///     - BLF's normalized parts inherit the source polygon's winding (no
    ///       ToCounterClockwise pass — see NestingEngine.NestBLF).
    ///     - PolygonInflate.Inflate preserves the input's winding sign.
    ///     - FinalVerifier sees the PlacedPolygon, which is CCW for the NFP path
    ///       but mixed-winding for the BLF path.
    ///   NonZero handles convex/concave/holed inputs correctly regardless of winding.
    ///
    ///   Do NOT change to FillRule.Positive without first enforcing CCW everywhere
    ///   upstream of OverlapChecker — Positive treats CW-wound polygons as empty and
    ///   would silently make Overlaps return false for any CW input. The NFP pipeline
    ///   in NoFitPolygon and NfpPlacementEngine deliberately uses Positive for the
    ///   opposite reason (CCW-guaranteed inputs, holes encoded as CW). The asymmetry
    ///   is intentional; see those files for the matching half of this argument.
    /// </summary>
    public static class OverlapChecker
    {
        /// <summary>
        /// Scale factor used when converting doubles to Clipper's int64 coordinate space.
        /// 10,000 gives 0.0001-unit resolution with ~922 trillion units of working headroom.
        /// </summary>
        public const double ClipperScale = 10000.0;

        /// <summary>
        /// Default linear tolerance for overlap detection, in model units.
        /// Intersections smaller than <c>tolerance * tolerance</c> (by area) are treated as non-overlapping.
        /// 0.01 is generous enough to absorb Clipper's integer-space edge-touch noise while
        /// still being far tighter than any real sheet-metal tolerance.
        /// </summary>
        public const double DefaultTolerance = 0.01;

        /// <summary>
        /// Return true if polygons <paramref name="a"/> and <paramref name="b"/> overlap by more than
        /// a <paramref name="tolerance"/>-sized sliver.
        /// </summary>
        public static bool Overlaps(Polygon a, Polygon b, double tolerance = DefaultTolerance)
        {
            double area = IntersectionArea(a, b);
            double areaThreshold = tolerance * tolerance;
            return area > areaThreshold;
        }

        /// <summary>
        /// Return the area of the geometric intersection of <paramref name="a"/> and
        /// <paramref name="b"/> in model units. Zero if the bounding boxes don't overlap
        /// or Clipper finds no intersection. Used by <see cref="Overlaps"/> for the
        /// verdict and exposed separately so diagnostic code can read the raw area
        /// alongside the threshold-based verdict.
        /// </summary>
        public static double IntersectionArea(Polygon a, Polygon b)
        {
            if (!a.BoundingBox.Intersects(b.BoundingBox))
                return 0.0;

            var subject = new Paths64 { ToPath64(a) };
            var clip = new Paths64 { ToPath64(b) };

            var solution = Clipper.Intersect(subject, clip, FillRule.NonZero);

            if (solution == null || solution.Count == 0)
                return 0.0;

            double scaledArea = 0.0;
            for (int i = 0; i < solution.Count; i++)
            {
                double a2 = Clipper.Area(solution[i]);
                if (a2 < 0) a2 = -a2;
                scaledArea += a2;
            }

            return scaledArea / (ClipperScale * ClipperScale);
        }

        /// <summary>
        /// Return true if <paramref name="candidate"/> overlaps any polygon in <paramref name="others"/>.
        /// Short-circuits on the first overlap found.
        /// </summary>
        public static bool OverlapsAny(Polygon candidate, IEnumerable<Polygon> others, double tolerance = DefaultTolerance)
        {
            foreach (var other in others)
            {
                if (Overlaps(candidate, other, tolerance))
                    return true;
            }
            return false;
        }

        private static Path64 ToPath64(Polygon poly)
        {
            var path = new Path64(poly.Count);
            var pts = poly.Points;
            for (int i = 0; i < pts.Count; i++)
            {
                long x = (long)(pts[i].X * ClipperScale);
                long y = (long)(pts[i].Y * ClipperScale);
                path.Add(new Point64(x, y));
            }
            return path;
        }
    }
}