using System;
using Rhino.Geometry;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Nesting;

namespace SeaNest.RhinoAdapters
{
    /// <summary>
    /// Converts a 2D <see cref="Polygon"/> to a Rhino <see cref="PolylineCurve"/> for drawing.
    ///
    /// Supports an optional per-sheet Y offset so multi-sheet nests stack vertically in the
    /// Rhino document without the caller having to remember to shift each placed polygon.
    /// This is the fix for the prior build's "all parts piled on sheet 0" bug — the offset
    /// is applied here, at the single Polygon→Curve boundary, rather than scattered across
    /// the draw loop.
    ///
    /// For Phase 2 NFP placements with mirror support, two distinct draw paths are available:
    ///
    ///   1. Draw from the placed polygon (<see cref="ToCurve(Polygon, double)"/>): the
    ///      polygon already has rotation, mirror, and translation baked in — fast and simple.
    ///      Use this when only the outline matters (e.g. nest visualization, sheet preview).
    ///
    ///   2. Draw from the original Rhino curve
    ///      (<see cref="ToCurveFromOriginal(Curve, PlacementResult, double)"/>): preserves
    ///      the original curve's degree, knots, and any embedded text/dim/etch detail.
    ///      Applies mirror (if requested), rotation, and translation in the correct order.
    ///      Use this when the user wants the full original geometry on the cut sheet.
    /// </summary>
    public static class PolygonToCurve
    {
        /// <summary>
        /// Convert a polygon to a closed polyline curve on the world XY plane (Z = 0).
        /// </summary>
        public static PolylineCurve ToCurve(Polygon polygon)
            => ToCurve(polygon, 0.0);

        /// <summary>
        /// Convert a polygon to a closed polyline curve, translated by <paramref name="yOffset"/>
        /// in Y. Use this for per-sheet vertical stacking (yOffset = Sheet * (sheetH + labelStride)).
        /// </summary>
        public static PolylineCurve ToCurve(Polygon polygon, double yOffset)
        {
            if (polygon == null) throw new ArgumentNullException(nameof(polygon));

            var pts = polygon.Points;
            int n = pts.Count;

            // Closed polyline needs the first point repeated at the end.
            var polyline = new Polyline(n + 1);
            for (int i = 0; i < n; i++)
            {
                polyline.Add(pts[i].X, pts[i].Y + yOffset, 0);
            }
            polyline.Add(pts[0].X, pts[0].Y + yOffset, 0);

            return new PolylineCurve(polyline);
        }

        /// <summary>
        /// Build the rectangular sheet outline as a closed polyline curve.
        /// </summary>
        /// <param name="sheetWidth">Sheet width.</param>
        /// <param name="sheetHeight">Sheet height.</param>
        /// <param name="yOffset">Y offset (for multi-sheet vertical stacking).</param>
        public static PolylineCurve SheetRectangle(double sheetWidth, double sheetHeight, double yOffset)
        {
            if (sheetWidth <= 0) throw new ArgumentException("Sheet width must be positive.", nameof(sheetWidth));
            if (sheetHeight <= 0) throw new ArgumentException("Sheet height must be positive.", nameof(sheetHeight));

            var polyline = new Polyline(5)
            {
                new Point3d(0,          yOffset,               0),
                new Point3d(sheetWidth, yOffset,               0),
                new Point3d(sheetWidth, yOffset + sheetHeight, 0),
                new Point3d(0,          yOffset + sheetHeight, 0),
                new Point3d(0,          yOffset,               0)
            };

            return new PolylineCurve(polyline);
        }

        /// <summary>
        /// Apply a <see cref="PlacementResult"/> to an original Rhino curve, producing a
        /// transformed curve ready to draw on the nest sheet. Preserves the original
        /// curve's geometric type and detail (degree, knots, fillets, etc.) — unlike
        /// <see cref="ToCurve(Polygon, double)"/> which only carries the flattened outline.
        ///
        /// Transform order (matches the <see cref="PlacementResult.IsMirrored"/> contract):
        ///   1. Mirror across the curve's local Y axis (i.e. world X-flip about the curve's
        ///      bbox-min corner) — only if <see cref="PlacementResult.IsMirrored"/>.
        ///   2. The rigid <see cref="PlacementResult.Transform"/> (rotation + translation).
        ///   3. Per-sheet Y offset for vertical stacking.
        ///
        /// The mirror is applied BEFORE the rigid transform because the rigid transform
        /// was constructed under the assumption that its input had been mirrored already
        /// (see <see cref="OrientedPart.Build"/>'s composition order: mirror → rotate →
        /// translate). Reversing the order would produce a placement off by twice the
        /// part's bbox width.
        /// </summary>
        /// <param name="originalCurve">
        /// The original 2D Rhino curve as it appeared on the user's input layer. Must be
        /// in the same coordinate frame as the <see cref="Polygon"/> that produced the
        /// placement (i.e., flattened to XY).
        /// </param>
        /// <param name="placement">
        /// Placement result from the nesting engine. Carries
        /// <see cref="PlacementResult.IsMirrored"/>, <see cref="PlacementResult.Transform"/>,
        /// and the source bounding-box info needed to mirror about the right axis.
        /// </param>
        /// <param name="yOffset">Per-sheet Y offset for multi-sheet vertical stacking.</param>
        public static Curve ToCurveFromOriginal(Curve originalCurve, PlacementResult placement, double yOffset)
        {
            if (originalCurve == null) throw new ArgumentNullException(nameof(originalCurve));
            if (placement == null) throw new ArgumentNullException(nameof(placement));

            // Duplicate so we don't mutate the user's input.
            var working = originalCurve.DuplicateCurve();
            if (working == null)
                throw new InvalidOperationException("Failed to duplicate original curve.");

            // Step 1: Mirror, if the placement is mirrored. The mirror axis is the line
            // x = bboxMinX in the curve's own coordinate frame — equivalently, the
            // X-flip-about-bbox-min that OrientedPart.Build applied to the polygon. This
            // matches the convention in Polygon.Mirror() (X-flip + winding reversal).
            //
            // Rhino's Transform.Mirror takes a plane; we want a vertical plane through
            // (bboxMinX, 0, 0) with normal = +X.
            if (placement.IsMirrored)
            {
                var bbox = working.GetBoundingBox(true);
                double mirrorX = bbox.Min.X;
                var mirrorPlane = new Plane(
                    new Point3d(mirrorX, 0, 0),
                    Vector3d.XAxis);
                var mirrorXform = Transform.Mirror(mirrorPlane);
                working.Transform(mirrorXform);
            }

            // Step 2: Apply the rigid Transform2D (rotation + translation) as a Rhino
            // 4×4 transform on the world XY plane.
            var rigid = ToRhinoTransform(placement.Transform);
            working.Transform(rigid);

            // Step 3: Per-sheet Y offset.
            if (yOffset != 0.0)
            {
                working.Transform(Transform.Translation(0, yOffset, 0));
            }

            return working;
        }

        /// <summary>
        /// Convenience overload with no Y offset.
        /// </summary>
        public static Curve ToCurveFromOriginal(Curve originalCurve, PlacementResult placement)
            => ToCurveFromOriginal(originalCurve, placement, 0.0);

        /// <summary>
        /// Convert a 2D rigid <see cref="Transform2D"/> (rotation + translation) into a
        /// Rhino 4×4 <see cref="Transform"/> that operates on the world XY plane (Z
        /// passed through unchanged).
        ///
        /// The 2D transform's matrix is:
        ///     [ cos  -sin  Tx ]
        ///     [ sin   cos  Ty ]
        ///     [  0     0    1 ]
        /// embedded into Rhino's 4×4 as:
        ///     [ cos  -sin  0  Tx ]
        ///     [ sin   cos  0  Ty ]
        ///     [  0     0   1   0 ]
        ///     [  0     0   0   1 ]
        /// </summary>
        private static Transform ToRhinoTransform(Transform2D t2d)
        {
            var x = Transform.Identity;
            x.M00 = t2d.Cos;
            x.M01 = -t2d.Sin;
            x.M03 = t2d.Tx;
            x.M10 = t2d.Sin;
            x.M11 = t2d.Cos;
            x.M13 = t2d.Ty;
            // M22 and M33 are 1.0 from Identity, M02/M12/M20/M21/M23/M30/M31/M32 are 0.0.
            return x;
        }
    }
}