using System;
using System.Collections.Generic;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// One oriented variant of a part: a specific (rotation, mirror) combination with the
    /// transformation already baked into the canonical polygon. Immutable.
    ///
    /// The NFP engine uses these as its working unit. Each <see cref="OrientedPart"/>
    /// has a unique <see cref="OrientationIndex"/> across the whole job; the NFP cache
    /// is keyed by pairs of these indices, so the cache is mirror- and rotation-blind
    /// once the orientations are constructed.
    ///
    /// Construction is the only place the rotation/mirror combination is materialised.
    /// Downstream code never re-applies the rotation or the mirror — everything is in
    /// <see cref="CanonicalPolygon"/>. At placement time, the engine emits a translation
    /// only; the original rotation and mirror are recovered for the <see cref="PlacementResult"/>
    /// from the <see cref="RotationDeg"/> and <see cref="IsMirrored"/> fields here.
    ///
    /// Reference-point convention: <see cref="CanonicalPolygon"/> has its bounding-box
    /// minimum corner at the origin. NFP placement returns translations relative to that
    /// reference point, which keeps IFP arithmetic simple (a rectangular sheet inset by
    /// the part's bbox extents from the reference point).
    /// </summary>
    public sealed class OrientedPart
    {
        /// <summary>
        /// Unique identifier for this orientation across all parts in the job.
        /// Used as half of the NFP cache key.
        /// </summary>
        public int OrientationIndex { get; }

        /// <summary>
        /// Index of the source part in <see cref="NestRequest.Polygons"/>. Lets the
        /// engine emit a <see cref="PlacementResult"/> referencing the original Rhino
        /// object after placement.
        /// </summary>
        public int SourcePartIndex { get; }

        /// <summary>
        /// Rotation applied to the source polygon, in degrees, CCW positive. Mirror
        /// (if any) was applied BEFORE this rotation — see <see cref="Build"/> for the
        /// composition order.
        /// </summary>
        public double RotationDeg { get; }

        /// <summary>
        /// True if the source polygon was X-flipped via <see cref="Polygon.Mirror"/>
        /// before rotation. Already baked into <see cref="CanonicalPolygon"/>.
        /// </summary>
        public bool IsMirrored { get; }

        /// <summary>
        /// The fully-transformed polygon: source mirrored (if requested), rotated, then
        /// translated so its bounding-box min corner is at the origin. Counter-clockwise
        /// winding (normalized for Clipper).
        /// </summary>
        public Polygon CanonicalPolygon { get; }

        /// <summary>
        /// Bounding box of <see cref="CanonicalPolygon"/>. Cached on the polygon itself
        /// but exposed here for direct access without forcing a property read on hot paths.
        /// Min corner is at (0, 0) by construction; <see cref="BoundingBox2D.MaxX"/> and
        /// <see cref="BoundingBox2D.MaxY"/> give the part's width and height in this orientation.
        /// </summary>
        public BoundingBox2D BBox { get; }

        private OrientedPart(
            int orientationIndex,
            int sourcePartIndex,
            double rotationDeg,
            bool isMirrored,
            Polygon canonicalPolygon)
        {
            OrientationIndex = orientationIndex;
            SourcePartIndex = sourcePartIndex;
            RotationDeg = rotationDeg;
            IsMirrored = isMirrored;
            CanonicalPolygon = canonicalPolygon;
            BBox = canonicalPolygon.BoundingBox;
        }

        /// <summary>
        /// Build a single oriented variant.
        ///
        /// Composition order: mirror (X-flip) → rotate around origin → translate so bbox
        /// min is at (0, 0) → normalize to counter-clockwise. The order matters: rotating
        /// before mirroring would yield a different shape (rotation and reflection don't
        /// commute), and the convention here matches what <see cref="PlacementResult.IsMirrored"/>
        /// promises downstream consumers (mirror-then-rotate-then-translate).
        /// </summary>
        public static OrientedPart Build(
            int orientationIndex,
            int sourcePartIndex,
            Polygon source,
            double rotationDeg,
            bool isMirrored)
        {
            if (source == null) throw new ArgumentNullException(nameof(source));

            Polygon working = source;
            if (isMirrored)
                working = working.Mirror();   // X-flip + winding reversal in one pass

            double radians = rotationDeg * Math.PI / 180.0;
            if (radians != 0.0)
                working = working.RotateAround(Point2D.Origin, radians);

            working = working.MoveToOrigin();
            working = working.ToCounterClockwise();

            // Hard invariant: every CanonicalPolygon must be CCW. The entire NFP pipeline
            // (NoFitPolygon, NfpPlacementEngine) standardizes on FillRule.Positive on the
            // strength of this guarantee. A CCW failure here means the source polygon was
            // degenerate (zero signed area, e.g. all-collinear points) and ToCounterClockwise
            // had nothing to flip. Failing fast with a clear message is far better than
            // letting Positive-rule Clipper ops silently produce empty forbidden regions
            // downstream and corrupt the nest.
            if (!working.IsCounterClockwise)
            {
                throw new InvalidOperationException(
                    $"OrientedPart.Build: source part {sourcePartIndex} produced a non-CCW canonical " +
                    $"polygon after normalization (signed area = {working.Area:G6}). " +
                    "This usually means the input polygon is degenerate (zero area / collinear vertices).");
            }

            return new OrientedPart(orientationIndex, sourcePartIndex, rotationDeg, isMirrored, working);
        }

        /// <summary>
        /// Build all oriented variants for a list of source polygons given a rotation step
        /// and mirror policy. Returns one flat list with stable orientation indices, plus
        /// a per-source-part lookup for the engine's outer loop.
        ///
        /// Orientation index assignment is sequential across the whole job, in this order:
        /// for each source part, for each rotation, original-then-mirrored. The exact ordering
        /// doesn't matter for correctness — the cache is symmetric in the index pair — but it
        /// must be deterministic so cache lookups stay stable across runs of the same input.
        /// </summary>
        /// <param name="sources">Source polygons, in <see cref="NestRequest.Polygons"/> order.</param>
        /// <param name="rotationStepDegrees">Step size for rotation enumeration.</param>
        /// <param name="allowMirror">When true, each rotation is emitted in original and X-flipped form.</param>
        /// <param name="allOrientations">Flat list of every <see cref="OrientedPart"/> produced.</param>
        /// <param name="byPartIndex">For each source part index, the orientations emitted for that part.</param>
        public static void BuildAll(
            IReadOnlyList<Polygon> sources,
            double rotationStepDegrees,
            bool allowMirror,
            out List<OrientedPart> allOrientations,
            out List<List<OrientedPart>> byPartIndex)
        {
            if (sources == null) throw new ArgumentNullException(nameof(sources));
            if (rotationStepDegrees <= 0.0)
                throw new ArgumentException("Rotation step must be positive.", nameof(rotationStepDegrees));

            int rotationCount = (int)Math.Round(360.0 / rotationStepDegrees);
            if (rotationCount < 1) rotationCount = 1;

            allOrientations = new List<OrientedPart>(sources.Count * rotationCount * (allowMirror ? 2 : 1));
            byPartIndex = new List<List<OrientedPart>>(sources.Count);

            int nextIndex = 0;

            for (int p = 0; p < sources.Count; p++)
            {
                var perPart = new List<OrientedPart>(rotationCount * (allowMirror ? 2 : 1));
                var sourcePoly = sources[p];

                for (int r = 0; r < rotationCount; r++)
                {
                    double rotDeg = r * rotationStepDegrees;

                    var orig = Build(nextIndex++, p, sourcePoly, rotDeg, isMirrored: false);
                    perPart.Add(orig);
                    allOrientations.Add(orig);

                    if (allowMirror)
                    {
                        var mir = Build(nextIndex++, p, sourcePoly, rotDeg, isMirrored: true);
                        perPart.Add(mir);
                        allOrientations.Add(mir);
                    }
                }

                byPartIndex.Add(perPart);
            }
        }

        public override string ToString()
        {
            return IsMirrored
                ? $"OrientedPart(idx={OrientationIndex}, src={SourcePartIndex}, rot={RotationDeg:F1}°, mirrored, bbox=[{BBox.MaxX:F2}×{BBox.MaxY:F2}])"
                : $"OrientedPart(idx={OrientationIndex}, src={SourcePartIndex}, rot={RotationDeg:F1}°, bbox=[{BBox.MaxX:F2}×{BBox.MaxY:F2}])";
        }
    }
}