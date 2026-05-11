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
        private double? _principalAxisAngle;
        private double? _aspectRatio;

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
        /// Direction of the polygon's principal axis (axis of largest variance),
        /// in radians from world +X. Computed via PCA on the vertex set using the
        /// centroid as origin: <c>θ = atan2(2·Sxy, Sxx − Syy) / 2</c>, with Sxx,
        /// Syy, Sxy the standard 2D covariance components. Range
        /// <c>[−π/2, π/2]</c>. Computed lazily and cached.
        ///
        /// <para>
        /// Used by callers that need a "visual long axis" — e.g. label rotation
        /// for elongated parts where the part's long direction is what humans
        /// expect text to read along. PCA matches eye intuition for typical
        /// plate parts (long thin strips, fantail panels, etc.); it is not
        /// minimum-area-OBB and may differ from a rotating-calipers result for
        /// shapes with strong corner constraints (L-shapes etc.).
        /// </para>
        ///
        /// <para>
        /// <b>Reading-direction ambiguity for near-vertical axes.</b> The
        /// principal axis is a direction (line), so θ and θ+π are geometrically
        /// equivalent. The atan2 formula returns one of the two equivalents,
        /// determined by sign of Sxy. For a vertical strip this means the
        /// returned angle is either ≈ +π/2 or ≈ −π/2 — text rendered at that
        /// angle reads either bottom-to-top or top-to-bottom. The result is
        /// consistent per polygon (same shape always gets the same convention),
        /// but is not normalized to a single global convention. Callers that
        /// need a canonical reading direction should normalize at the call site.
        /// </para>
        ///
        /// <para>
        /// <b>Stability for near-square parts.</b> When Sxx ≈ Syy and Sxy ≈ 0
        /// (rotationally symmetric shapes, near-circular parts), the principal
        /// axis is geometrically indeterminate and small numerical perturbations
        /// can flip it by 90°. Callers should gate use of this property on a
        /// minimum <see cref="AspectRatio"/> threshold (e.g. > 2 or > 5) to
        /// avoid acting on noise.
        /// </para>
        /// </summary>
        public double PrincipalAxisAngle
        {
            get
            {
                if (!_principalAxisAngle.HasValue)
                    _principalAxisAngle = ComputePrincipalAxisAngle();
                return _principalAxisAngle.Value;
            }
        }

        /// <summary>
        /// Aspect ratio of the polygon as measured against its
        /// <see cref="PrincipalAxisAngle"/>: the extent along the principal axis
        /// divided by the extent along the perpendicular axis. Always ≥ 1.0 by
        /// construction (principal axis is the direction of largest spread).
        /// Computed lazily and cached.
        ///
        /// <para>
        /// Single O(n) pass projecting each vertex onto the principal and
        /// perpendicular unit vectors and tracking min/max on each. Uses the
        /// cached <see cref="PrincipalAxisAngle"/> (which uses cached
        /// <see cref="Centroid"/>), so first access of either property pays the
        /// PCA cost once and subsequent accesses are O(1).
        /// </para>
        ///
        /// <para>
        /// Used to decide whether <see cref="PrincipalAxisAngle"/> is meaningful
        /// enough to act on. Near 1.0 = essentially-square; > 5 = visually-
        /// elongated strip.
        /// </para>
        /// </summary>
        public double AspectRatio
        {
            get
            {
                if (!_aspectRatio.HasValue)
                    _aspectRatio = ComputeAspectRatio();
                return _aspectRatio.Value;
            }
        }

        /// <summary>
        /// Return the "pole of inaccessibility" — the point inside the polygon
        /// farthest from any edge. For convex polygons this is near the centroid;
        /// for concave polygons (notched plates, fantail panels with deep inward
        /// curves) it remains inside the solid body, where the centroid can fall
        /// outside. Useful as a visually-centered anchor for labels, dimensions,
        /// or any annotation that must land on the part itself rather than in a
        /// notch or beyond the outline.
        ///
        /// Algorithm: Mapbox Polylabel. Quadtree-style search via a max-heap of
        /// square cells keyed on each cell's maximum possible signed distance to
        /// the boundary (cell.d + cell.h·√2). Cells are subdivided into 4
        /// quadrants only while their potential can still beat the best-so-far
        /// by more than <paramref name="precision"/>. The centroid and the bbox
        /// center seed the initial candidate so the algorithm starts with a
        /// reasonable lower bound and converges fast.
        ///
        /// <para>
        /// Pre-allocates the heap with a small initial capacity; subdivision
        /// rarely produces more than a few hundred cells in practice. Per-cell
        /// cost is O(n) where n is the polygon vertex count (one pass over edges
        /// for signed distance + even-odd inside test). Total cost ~100µs-4ms per
        /// typical plate part; imperceptible against NFP compute time.
        /// </para>
        ///
        /// <para>
        /// <paramref name="precision"/> in model units. Values below 0.5" are
        /// invisible at typical label sizes; values above 5% of the polygon's
        /// smaller bbox extent start to drift the anchor noticeably. Standard
        /// choice is around 0.5"; the algorithm terminates when no remaining
        /// cell could improve the best by more than this.
        /// </para>
        ///
        /// <para>
        /// Returns <see cref="Centroid"/> as a fallback if the polygon's
        /// bounding box is degenerate (zero width or height).
        /// </para>
        /// </summary>
        public Point2D PoleOfInaccessibility(double precision)
        {
            if (precision <= 0)
                throw new ArgumentException("Precision must be positive.", nameof(precision));

            var bbox = BoundingBox;
            double width = bbox.MaxX - bbox.MinX;
            double height = bbox.MaxY - bbox.MinY;
            double cellSize = Math.Min(width, height);
            if (cellSize <= 0)
                return Centroid;

            double h = cellSize / 2.0;

            var queue = new PoleCellHeap(initialCapacity: 64);

            // Seed with a uniform grid of cellSize-square cells covering the
            // bbox. Cells whose centers fall outside the polygon get negative
            // signed distance; they're not discarded — their max-potential
            // (d + h·√2) may still be positive and subdivision may yield
            // children whose centers fall inside.
            for (double x = bbox.MinX; x < bbox.MaxX; x += cellSize)
            {
                for (double y = bbox.MinY; y < bbox.MaxY; y += cellSize)
                {
                    queue.Push(new PoleCell(x + h, y + h, h, this));
                }
            }

            // Initial candidate: centroid (often a good starting bound).
            var centroid = Centroid;
            var best = new PoleCell(centroid.X, centroid.Y, 0, this);

            // Bounding-box center as a fallback initial candidate — for
            // concave shapes whose centroid falls outside the polygon, the
            // bbox center is sometimes a better bound.
            var bboxCenter = new PoleCell(bbox.MinX + width / 2.0, bbox.MinY + height / 2.0, 0, this);
            if (bboxCenter.D > best.D) best = bboxCenter;

            while (queue.Count > 0)
            {
                var cell = queue.Pop();

                if (cell.D > best.D) best = cell;

                // Prune: this cell's best possible interior point can't
                // outperform the current best by more than precision.
                if (cell.Max - best.D <= precision) continue;

                double newH = cell.H / 2.0;
                queue.Push(new PoleCell(cell.X - newH, cell.Y - newH, newH, this));
                queue.Push(new PoleCell(cell.X + newH, cell.Y - newH, newH, this));
                queue.Push(new PoleCell(cell.X - newH, cell.Y + newH, newH, this));
                queue.Push(new PoleCell(cell.X + newH, cell.Y + newH, newH, this));
            }

            return new Point2D(best.X, best.Y);
        }

        /// <summary>
        /// Signed distance from <c>(px, py)</c> to the polygon's boundary. Positive
        /// inside the polygon, negative outside. Single pass over edges that
        /// simultaneously computes the minimum point-to-segment distance and
        /// runs the even-odd ray-crossing inside test — efficient for
        /// <see cref="PoleOfInaccessibility"/>'s per-cell evaluation.
        /// </summary>
        private double SignedDistanceToBoundary(double px, double py)
        {
            double minDistSq = double.MaxValue;
            bool inside = false;
            int n = _points.Length;

            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                var a = _points[i];
                var b = _points[j];

                // Even-odd inside test: count edges crossed by a +X ray from (px, py).
                // Edge (a, b) crosses the horizontal line y = py iff a.Y and b.Y
                // straddle py; the crossing's X is computed by linear interpolation.
                if ((a.Y > py) != (b.Y > py) &&
                    px < (b.X - a.X) * (py - a.Y) / (b.Y - a.Y) + a.X)
                {
                    inside = !inside;
                }

                double dsq = DistSqToSegment(px, py, a, b);
                if (dsq < minDistSq) minDistSq = dsq;
            }

            double d = Math.Sqrt(minDistSq);
            return inside ? d : -d;
        }

        /// <summary>
        /// Squared distance from a point to a line segment in 2D. Standard
        /// projection-then-clamp implementation; returns squared distance to
        /// avoid an unnecessary sqrt per edge in the
        /// <see cref="SignedDistanceToBoundary"/> hot loop (sqrt is taken once
        /// after the min is known).
        /// </summary>
        private static double DistSqToSegment(double px, double py, Point2D a, Point2D b)
        {
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            double lenSq = dx * dx + dy * dy;

            if (lenSq == 0.0)
            {
                double pdx = px - a.X;
                double pdy = py - a.Y;
                return pdx * pdx + pdy * pdy;
            }

            double t = ((px - a.X) * dx + (py - a.Y) * dy) / lenSq;
            if (t < 0.0) t = 0.0;
            else if (t > 1.0) t = 1.0;

            double cx = a.X + t * dx;
            double cy = a.Y + t * dy;
            double ex = px - cx;
            double ey = py - cy;
            return ex * ex + ey * ey;
        }

        /// <summary>
        /// One square cell in <see cref="PoleOfInaccessibility"/>'s quadtree
        /// search. Center <c>(X, Y)</c>, half-size <c>H</c>, signed distance
        /// from center to boundary <c>D</c>, and <c>Max = D + H·√2</c> — the
        /// upper bound on signed distance for any point inside this cell
        /// (corner is at distance H·√2 from center). Immutable; constructed
        /// at push time so the heap comparator reads cached <c>Max</c>.
        /// </summary>
        private readonly struct PoleCell
        {
            public readonly double X;
            public readonly double Y;
            public readonly double H;
            public readonly double D;
            public readonly double Max;

            public PoleCell(double x, double y, double h, Polygon poly)
            {
                X = x;
                Y = y;
                H = h;
                D = poly.SignedDistanceToBoundary(x, y);
                Max = D + h * Math.Sqrt(2.0);
            }
        }

        /// <summary>
        /// Bespoke max-heap of <see cref="PoleCell"/>s keyed on
        /// <see cref="PoleCell.Max"/>. Array-backed, pre-allocated to a small
        /// capacity to avoid early List grows. .NET 6+ has
        /// <c>System.Collections.Generic.PriorityQueue</c> but this lib targets
        /// netstandard2.0 which doesn't, so we ship our own ~30-line heap.
        ///
        /// Standard binary-heap algorithm: sift-up on <see cref="Push"/>,
        /// sift-down on <see cref="Pop"/>. Both O(log n). The comparator is
        /// "highest Max wins": parents have >= Max than children.
        /// </summary>
        private sealed class PoleCellHeap
        {
            private readonly List<PoleCell> _items;

            public PoleCellHeap(int initialCapacity)
            {
                _items = new List<PoleCell>(initialCapacity);
            }

            public int Count => _items.Count;

            public void Push(PoleCell cell)
            {
                _items.Add(cell);
                SiftUp(_items.Count - 1);
            }

            public PoleCell Pop()
            {
                var top = _items[0];
                int last = _items.Count - 1;
                _items[0] = _items[last];
                _items.RemoveAt(last);
                if (_items.Count > 0) SiftDown(0);
                return top;
            }

            private void SiftUp(int i)
            {
                while (i > 0)
                {
                    int parent = (i - 1) / 2;
                    if (_items[i].Max <= _items[parent].Max) break;
                    var tmp = _items[i];
                    _items[i] = _items[parent];
                    _items[parent] = tmp;
                    i = parent;
                }
            }

            private void SiftDown(int i)
            {
                int n = _items.Count;
                while (true)
                {
                    int left = 2 * i + 1;
                    int right = 2 * i + 2;
                    int best = i;
                    if (left < n && _items[left].Max > _items[best].Max) best = left;
                    if (right < n && _items[right].Max > _items[best].Max) best = right;
                    if (best == i) break;
                    var tmp = _items[i];
                    _items[i] = _items[best];
                    _items[best] = tmp;
                    i = best;
                }
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
        /// Simplify to a target maximum vertex count using <see cref="SimplifyDouglasPeucker"/>
        /// with tolerance escalation. Always runs at least one DP pass at
        /// <paramref name="initialTolerance"/>; if the result still exceeds
        /// <paramref name="maxVertices"/>, multiplies tolerance by
        /// <paramref name="escalationFactor"/> and retries — re-running on the ORIGINAL
        /// polygon each time, not on the previously-simplified result, because DP is
        /// deterministic given (input, tolerance) and re-simplifying already-simplified
        /// geometry produces a different (worse) outcome than simplifying the original
        /// at the higher tolerance directly.
        ///
        /// Stops at the first iteration where Count ≤ maxVertices, OR after
        /// <paramref name="maxEscalations"/> escalation passes regardless of count
        /// (returning the over-cap result rather than failing — caller should log).
        ///
        /// <paramref name="finalTolerance"/> reports the tolerance of the returned
        /// result; comparing it to <paramref name="initialTolerance"/> tells the
        /// caller whether escalation fired.
        /// </summary>
        public Polygon SimplifyToTarget(
            double initialTolerance,
            int maxVertices,
            double escalationFactor,
            int maxEscalations,
            out double finalTolerance)
        {
            if (initialTolerance <= 0)
                throw new ArgumentException("initialTolerance must be positive.", nameof(initialTolerance));
            if (maxVertices < 3)
                throw new ArgumentException("maxVertices must be at least 3.", nameof(maxVertices));
            if (escalationFactor <= 1.0)
                throw new ArgumentException("escalationFactor must be greater than 1.0.", nameof(escalationFactor));
            if (maxEscalations < 0)
                throw new ArgumentException("maxEscalations must be non-negative.", nameof(maxEscalations));

            double tol = initialTolerance;
            Polygon result = SimplifyDouglasPeucker(tol);

            int escalations = 0;
            while (result.Count > maxVertices && escalations < maxEscalations)
            {
                tol *= escalationFactor;
                result = SimplifyDouglasPeucker(tol); // run on original (this), not on result
                escalations++;
            }

            finalTolerance = tol;
            return result;
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

        private double ComputePrincipalAxisAngle()
        {
            // 2D PCA on the vertex set, weighted uniformly. Returns the angle of
            // the eigenvector corresponding to the larger eigenvalue of the
            // covariance matrix [[Sxx, Sxy], [Sxy, Syy]].
            //
            // The closed-form for the principal-axis angle in 2D is
            //     tan(2θ) = 2·Sxy / (Sxx − Syy)
            // so θ = atan2(2·Sxy, Sxx − Syy) / 2.
            // atan2 returns the angle in (−π, π]; dividing by 2 puts θ in
            // (−π/2, π/2]. By the construction of the atan2 arguments this is
            // always the larger-eigenvalue axis (the longer direction).
            //
            // Degenerate (Sxx == Syy and Sxy == 0): atan2(0, 0) returns 0 and
            // we return 0. This case only arises for rotationally-symmetric
            // vertex sets; callers should gate use on AspectRatio anyway.
            var c = Centroid;
            double sxx = 0.0, syy = 0.0, sxy = 0.0;
            for (int i = 0; i < _points.Length; i++)
            {
                double dx = _points[i].X - c.X;
                double dy = _points[i].Y - c.Y;
                sxx += dx * dx;
                syy += dy * dy;
                sxy += dx * dy;
            }
            return Math.Atan2(2.0 * sxy, sxx - syy) * 0.5;
        }

        private double ComputeAspectRatio()
        {
            // Project every vertex onto the principal axis (cosθ, sinθ) and the
            // perpendicular axis (−sinθ, cosθ); take the (max−min) extent on
            // each. The principal extent is, by construction of PCA, >= the
            // perpendicular extent, so the ratio is always >= 1.
            //
            // For a degenerate (zero-extent) perpendicular — collinear vertex
            // set, which Polygon's constructor already rejects in practice —
            // return double.PositiveInfinity so any reasonable threshold
            // check still rejects the geometry rather than producing NaN.
            double theta = PrincipalAxisAngle;
            double cos = Math.Cos(theta);
            double sin = Math.Sin(theta);

            double minU = double.PositiveInfinity, maxU = double.NegativeInfinity;
            double minV = double.PositiveInfinity, maxV = double.NegativeInfinity;
            for (int i = 0; i < _points.Length; i++)
            {
                double x = _points[i].X;
                double y = _points[i].Y;
                double u = x * cos + y * sin;          // projection onto principal
                double v = -x * sin + y * cos;         // projection onto perpendicular
                if (u < minU) minU = u;
                if (u > maxU) maxU = u;
                if (v < minV) minV = v;
                if (v > maxV) maxV = v;
            }
            double length = maxU - minU;
            double width = maxV - minV;
            if (width <= 0.0) return double.PositiveInfinity;
            return length / width;
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