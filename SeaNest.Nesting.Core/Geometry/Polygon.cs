using System;
using System.Collections.Generic;
using System.Linq;

namespace SeaNest.Nesting.Core.Geometry
{
    /// <summary>
    /// A closed 2D polygon represented as an ordered list of vertices.
    /// The Nth edge implicitly closes from Points[N-1] back to Points[0].
    /// Points are never duplicated at the end (Points[0] != Points[Count-1]).
    ///
    /// Polygons are immutable externally — Transform/Translate/Rotate return NEW polygons.
    /// Cached properties (BoundingBox, Area, Centroid) compute lazily on first access.
    /// </summary>
    public sealed class Polygon
    {
        private readonly Point2D[] _points;

        // Cached lazily — null means not yet computed
        private BoundingBox2D? _bbox;
        private double? _area;
        private Point2D? _centroid;
        private bool? _isConvex;

        /// <summary>
        /// Create a polygon from a sequence of points.
        /// If the first and last points are equal (within 1e-9), the duplicate end point is removed.
        /// Requires at least 3 distinct points.
        /// </summary>
        public Polygon(IEnumerable<Point2D> points)
        {
            if (points == null) throw new ArgumentNullException(nameof(points));

            var list = points.ToList();
            if (list.Count < 3)
                throw new ArgumentException(
                    $"Polygon requires at least 3 points; got {list.Count}", nameof(points));

            // Remove trailing duplicate of first point if present
            if (list.Count >= 2 && list[0].EqualsWithTolerance(list[list.Count - 1], 1e-9))
            {
                list.RemoveAt(list.Count - 1);
            }

            if (list.Count < 3)
                throw new ArgumentException(
                    $"Polygon requires at least 3 distinct points after deduplication; got {list.Count}",
                    nameof(points));

            _points = list.ToArray();
        }

        /// <summary>
        /// Private constructor for internal fast-path operations that already have a valid array.
        /// Skips validation and duplicate removal.
        /// </summary>
        private Polygon(Point2D[] points, bool _unused)
        {
            _points = points;
        }

        /// <summary>
        /// Read-only access to the polygon's vertices.
        /// </summary>
        public IReadOnlyList<Point2D> Points => _points;

        /// <summary>
        /// Number of vertices (equals number of edges since the loop is closed).
        /// </summary>
        public int Count => _points.Length;

        /// <summary>
        /// Axis-aligned bounding box of the polygon. Computed lazily and cached.
        /// </summary>
        public BoundingBox2D BoundingBox
        {
            get
            {
                if (!_bbox.HasValue)
                    _bbox = ComputeBoundingBox();
                return _bbox.Value;
            }
        }

        /// <summary>
        /// Signed area of the polygon. Positive for counter-clockwise winding, negative for clockwise.
        /// Use Math.Abs(Area) for the unsigned area. Computed lazily and cached.
        /// </summary>
        public double Area
        {
            get
            {
                if (!_area.HasValue)
                    _area = ComputeSignedArea();
                return _area.Value;
            }
        }

        /// <summary>
        /// Absolute (unsigned) area of the polygon.
        /// </summary>
        public double AbsoluteArea => Math.Abs(Area);

        /// <summary>
        /// True if the polygon is wound counter-clockwise (standard orientation).
        /// </summary>
        public bool IsCounterClockwise => Area > 0;

        /// <summary>
        /// Area-weighted centroid of the polygon. Computed lazily and cached.
        /// </summary>
        public Point2D Centroid
        {
            get
            {
                if (!_centroid.HasValue)
                    _centroid = ComputeCentroid();
                return _centroid.Value;
            }
        }

        /// <summary>
        /// True if the polygon is convex — every interior angle ≤ 180° (winding-
        /// agnostic: works for both CCW and CW polygons). Computed lazily and
        /// cached.
        ///
        /// Used by the NFP pipeline to short-circuit the concave-aware CW filter
        /// when both inputs are convex: the Minkowski difference of two convex
        /// polygons is mathematically convex (single CCW outer boundary, no
        /// holes), so any CW sub-loop in the Union output is necessarily a
        /// numerical artifact and gets blanket-reversed without invoking the
        /// area+PIP classifier (which can't distinguish convex-input artifacts
        /// from real concave-input holes by spatial criteria alone).
        ///
        /// Self-intersecting polygons (figure-8) can pass this test but aren't
        /// produced by any caller — Polygon doesn't validate simplicity.
        /// </summary>
        public bool IsConvex
        {
            get
            {
                if (!_isConvex.HasValue)
                    _isConvex = ComputeIsConvex();
                return _isConvex.Value;
            }
        }

        /// <summary>
        /// Return a new polygon translated by (dx, dy).
        /// </summary>
        public Polygon Translate(double dx, double dy)
        {
            var newPoints = new Point2D[_points.Length];
            for (int i = 0; i < _points.Length; i++)
                newPoints[i] = _points[i].Translate(dx, dy);
            return new Polygon(newPoints, true);
        }

        /// <summary>
        /// Return a new polygon rotated around a pivot point by the given angle.
        /// </summary>
        public Polygon RotateAround(Point2D pivot, double angleRadians)
        {
            var newPoints = new Point2D[_points.Length];
            for (int i = 0; i < _points.Length; i++)
                newPoints[i] = _points[i].RotateAround(pivot, angleRadians);
            return new Polygon(newPoints, true);
        }

        /// <summary>
        /// Return a new polygon rotated around its bounding-box center.
        /// </summary>
        public Polygon RotateAroundCenter(double angleRadians)
        {
            return RotateAround(BoundingBox.Center, angleRadians);
        }

        /// <summary>
        /// Return a new polygon with every vertex transformed by the given affine transform.
        /// Since <see cref="Transform2D"/> is a rigid transform (rotation + translation only,
        /// determinant = +1), winding order is preserved — no reversal needed afterwards.
        /// </summary>
        public Polygon Transform(Transform2D transform)
        {
            var newPoints = new Point2D[_points.Length];
            for (int i = 0; i < _points.Length; i++)
                newPoints[i] = transform.Apply(_points[i]);
            return new Polygon(newPoints, true);
        }

        /// <summary>
        /// Return a new polygon translated so its bounding-box min corner is at (0, 0).
        /// </summary>
        public Polygon MoveToOrigin()
        {
            var bb = BoundingBox;
            return Translate(-bb.MinX, -bb.MinY);
        }

        /// <summary>
        /// Return a new polygon with reversed winding order.
        /// </summary>
        public Polygon Reversed()
        {
            var newPoints = new Point2D[_points.Length];
            for (int i = 0; i < _points.Length; i++)
                newPoints[i] = _points[_points.Length - 1 - i];
            return new Polygon(newPoints, true);
        }

        /// <summary>
        /// Return a new polygon with counter-clockwise winding, reversing if needed.
        /// Useful for normalizing input before Clipper operations.
        /// </summary>
        public Polygon ToCounterClockwise()
        {
            return IsCounterClockwise ? this : Reversed();
        }

        /// <summary>
        /// Return a new polygon mirrored across the Y axis (x -> -x).
        ///
        /// Mirror is a non-rigid transform with determinant -1, so it reverses winding.
        /// This method reverses the vertex order to restore the original winding sense
        /// (a CCW input remains CCW on output) — correct for Clipper, NFP, and overlap code
        /// downstream.
        ///
        /// Used by the NFP placement engine to allow parts to be flipped left-to-right on the
        /// sheet — equivalent to a CAM "flip horizontal" operation. The flip propagates through
        /// to <see cref="PlacementResult.IsMirrored"/> and is reapplied to the original Rhino
        /// curves at draw time, not baked into the placement transform.
        /// </summary>
        public Polygon Mirror()
        {
            int n = _points.Length;
            var newPoints = new Point2D[n];
            // Flip x and reverse order in one pass: newPoints[i] = mirror(_points[n-1-i])
            for (int i = 0; i < n; i++)
            {
                var src = _points[n - 1 - i];
                newPoints[i] = new Point2D(-src.X, src.Y);
            }
            return new Polygon(newPoints, true);
        }

        /// <summary>
        /// Return a new polygon mirrored across the X axis (y -> -y).
        ///
        /// Provided for completeness; the NFP path uses <see cref="Mirror"/> (Y-axis flip) by
        /// convention. Mirror-X is equivalent to Mirror-Y composed with a 180° rotation, so
        /// using both in the same orientation enumerator is redundant.
        ///
        /// Reverses vertex order for the same winding-preservation reason as <see cref="Mirror"/>.
        /// </summary>
        public Polygon MirrorY()
        {
            int n = _points.Length;
            var newPoints = new Point2D[n];
            for (int i = 0; i < n; i++)
            {
                var src = _points[n - 1 - i];
                newPoints[i] = new Point2D(src.X, -src.Y);
            }
            return new Polygon(newPoints, true);
        }

        /// <summary>
        /// Return a new polygon with near-collinear vertices removed. A vertex is removed
        /// if its perpendicular distance to the line connecting its two neighbors is below
        /// <paramref name="tolerance"/>. Iteratively picks the vertex with the smallest
        /// deviation each pass, so adjacent vertices are never removed simultaneously
        /// (which would produce different output than removing them one at a time).
        ///
        /// Use this before any expensive geometric operation (NFP, ear-clipping, Minkowski).
        /// Squish-meshed polygons in particular contain dozens of near-collinear vertices
        /// along originally-straight edges that contribute nothing to the polygon's shape
        /// but multiply downstream cost.
        ///
        /// Tolerance should be well below any real-world cutting tolerance — 5× the model
        /// absolute tolerance is a good default for sheet-metal work.
        /// </summary>
        public Polygon Simplify(double tolerance)
        {
            if (tolerance <= 0 || _points.Length <= 3)
                return this;
            var pts = new List<Point2D>(_points);
            double tolSq = tolerance * tolerance;

            while (pts.Count > 3)
            {
                int worstIdx = -1;
                double worstDistSq = double.MaxValue;
                int n = pts.Count;

                for (int i = 0; i < n; i++)
                {
                    var prev = pts[(i - 1 + n) % n];
                    var pt = pts[i];
                    var nxt = pts[(i + 1) % n];

                    double dx = nxt.X - prev.X;
                    double dy = nxt.Y - prev.Y;
                    double len2 = dx * dx + dy * dy;
                    if (len2 < 1e-24) continue; // prev == next, skip

                    // Signed perpendicular-distance squared from pt to line(prev, nxt).
                    double cross = (pt.X - prev.X) * dy - (pt.Y - prev.Y) * dx;
                    double distSq = (cross * cross) / len2;

                    if (distSq < worstDistSq)
                    {
                        worstDistSq = distSq;
                        worstIdx = i;
                    }
                }

                if (worstIdx < 0 || worstDistSq >= tolSq) break;
                pts.RemoveAt(worstIdx);
            }

            return new Polygon(pts);
        }

        /// <summary>
        /// Return a new polygon simplified using the Ramer-Douglas-Peucker algorithm.
        /// More aggressive than <see cref="Simplify"/>: where Simplify only removes
        /// near-collinear vertices, DP also removes vertices on arc approximations
        /// where the resulting chord stays within <paramref name="tolerance"/> of the
        /// original polyline.
        ///
        /// Use this for inputs where dense vertex runs approximate arcs (rat holes,
        /// slot fillets, hull curves). At sheet-metal CAM tolerances (0.005-0.01"),
        /// rat-hole arcs go from ~20 vertices to ~4 with no visible deviation.
        ///
        /// Slot tab corners are sharp angles and survive DP at any reasonable
        /// tolerance because the perpendicular distance from the corner vertex to
        /// the chord skipping it is large.
        /// </summary>
        public Polygon SimplifyDouglasPeucker(double tolerance)
        {
            if (tolerance <= 0 || _points.Length <= 3)
                return this;

            int n = _points.Length;

            // For a closed polygon, "open" it at the vertex farthest from the centroid.
            // This anchor vertex is guaranteed to be on the convex hull and is a stable
            // starting point that doesn't depend on input vertex ordering.
            int anchorIdx = FindFarthestFromCentroid();

            // Rotate the vertex list so anchor is at index 0, and append anchor again
            // at the end to form an open polyline that DP can process linearly.
            var opened = new Point2D[n + 1];
            for (int i = 0; i < n; i++)
                opened[i] = _points[(anchorIdx + i) % n];
            opened[n] = opened[0]; // close the loop

            // Run DP on the opened polyline. Mark which vertices to keep.
            var keep = new bool[n + 1];
            keep[0] = true;
            keep[n] = true;
            DpRecurse(opened, 0, n, tolerance, keep);

            // Collect kept vertices (excluding the duplicated final closing vertex).
            var kept = new List<Point2D>(n);
            for (int i = 0; i < n; i++)
                if (keep[i]) kept.Add(opened[i]);

            if (kept.Count < 3) return this; // safety: never reduce below triangle
            return new Polygon(kept);
        }

        /// <summary>
        /// Index of the vertex farthest from the polygon's centroid. Used to anchor
        /// DP recursion at a stable, geometry-invariant starting point.
        /// </summary>
        private int FindFarthestFromCentroid()
        {
            var c = Centroid;
            double bestDistSq = -1;
            int bestIdx = 0;
            for (int i = 0; i < _points.Length; i++)
            {
                double dx = _points[i].X - c.X;
                double dy = _points[i].Y - c.Y;
                double d2 = dx * dx + dy * dy;
                if (d2 > bestDistSq) { bestDistSq = d2; bestIdx = i; }
            }
            return bestIdx;
        }

        /// <summary>
        /// Standard Douglas-Peucker recursion on an open polyline indexed [lo..hi].
        /// Marks vertices to keep in <paramref name="keep"/>.
        /// </summary>
        private static void DpRecurse(Point2D[] pts, int lo, int hi, double tolerance, bool[] keep)
        {
            if (hi - lo < 2) return;

            var a = pts[lo];
            var b = pts[hi];
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            double lenSq = dx * dx + dy * dy;

            int worstIdx = -1;
            double worstDistSq = -1;

            for (int i = lo + 1; i < hi; i++)
            {
                double distSq;
                if (lenSq < 1e-24)
                {
                    // Degenerate: a == b. Use euclidean distance from a.
                    double ex = pts[i].X - a.X;
                    double ey = pts[i].Y - a.Y;
                    distSq = ex * ex + ey * ey;
                }
                else
                {
                    // Perpendicular distance from pts[i] to line through a in direction (dx, dy).
                    double cross = (pts[i].X - a.X) * dy - (pts[i].Y - a.Y) * dx;
                    distSq = (cross * cross) / lenSq;
                }

                if (distSq > worstDistSq)
                {
                    worstDistSq = distSq;
                    worstIdx = i;
                }
            }

            if (worstIdx >= 0 && worstDistSq > tolerance * tolerance)
            {
                keep[worstIdx] = true;
                DpRecurse(pts, lo, worstIdx, tolerance, keep);
                DpRecurse(pts, worstIdx, hi, tolerance, keep);
            }
        }

        // ------------------------------------------------------------------
        // Private computations
        // ------------------------------------------------------------------

        private BoundingBox2D ComputeBoundingBox()
        {
            double minX = _points[0].X, maxX = _points[0].X;
            double minY = _points[0].Y, maxY = _points[0].Y;

            for (int i = 1; i < _points.Length; i++)
            {
                var p = _points[i];
                if (p.X < minX) minX = p.X;
                if (p.X > maxX) maxX = p.X;
                if (p.Y < minY) minY = p.Y;
                if (p.Y > maxY) maxY = p.Y;
            }

            return new BoundingBox2D(minX, minY, maxX, maxY);
        }

        private double ComputeSignedArea()
        {
            // Shoelace formula
            double sum = 0;
            int n = _points.Length;
            for (int i = 0; i < n; i++)
            {
                var a = _points[i];
                var b = _points[(i + 1) % n];
                sum += (a.X * b.Y) - (b.X * a.Y);
            }
            return sum * 0.5;
        }

        private Point2D ComputeCentroid()
        {
            // Standard area-weighted polygon centroid formula
            double signedArea = Area;
            if (Math.Abs(signedArea) < 1e-12)
            {
                // Degenerate polygon; fall back to bounding box center
                return BoundingBox.Center;
            }

            double cx = 0, cy = 0;
            int n = _points.Length;
            for (int i = 0; i < n; i++)
            {
                var a = _points[i];
                var b = _points[(i + 1) % n];
                double cross = (a.X * b.Y) - (b.X * a.Y);
                cx += (a.X + b.X) * cross;
                cy += (a.Y + b.Y) * cross;
            }

            double factor = 1.0 / (6.0 * signedArea);
            return new Point2D(cx * factor, cy * factor);
        }

        private bool ComputeIsConvex()
        {
            // For each vertex, cross-product of the incoming edge with the
            // outgoing edge. Convex iff all non-zero crosses share a sign
            // (all positive for CCW, all negative for CW). Mixed signs ⇒
            // at least one reflex vertex ⇒ concave.
            //
            // Collinear vertices produce cross == 0; they don't pin a sign
            // and don't disqualify convexity. An all-collinear polygon
            // (Area == 0) returns true vacuously, but OrientedPart.Build
            // asserts CCW (Area > 0) upstream so this path doesn't fire on
            // engine inputs.
            //
            // Tolerance 1e-12 is well below any meaningful geometry on
            // model-unit polygons (typically inches) and well above
            // floating-point noise on translate/rotate of typical sizes.
            int n = _points.Length;
            if (n < 3) return false;
            if (n == 3) return true;

            const double eps = 1e-12;
            int sign = 0;

            for (int i = 0; i < n; i++)
            {
                var prev = _points[(i - 1 + n) % n];
                var curr = _points[i];
                var next = _points[(i + 1) % n];

                double e1x = curr.X - prev.X;
                double e1y = curr.Y - prev.Y;
                double e2x = next.X - curr.X;
                double e2y = next.Y - curr.Y;
                double cross = e1x * e2y - e1y * e2x;

                if (Math.Abs(cross) < eps) continue; // collinear vertex; sign-neutral

                int thisSign = cross > 0 ? 1 : -1;
                if (sign == 0) sign = thisSign;
                else if (sign != thisSign) return false;
            }

            return true;
        }

        public override string ToString()
        {
            var bb = BoundingBox;
            return $"Polygon(Count={_points.Length}, BBox=[{bb.MinX:F2},{bb.MinY:F2}]-[{bb.MaxX:F2},{bb.MaxY:F2}], Area={AbsoluteArea:F2})";
        }
    }
}