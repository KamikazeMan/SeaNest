using System;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// One placed part in a nest result. Immutable.
    ///
    /// Carries enough information to reconstruct the placement against the original
    /// Rhino curve at draw time, plus the already-transformed 2D polygon for direct
    /// rendering via <see cref="SeaNest.RhinoAdapters.PolygonToCurve"/>.
    ///
    /// The <see cref="PlacedPolygon"/> field already has rotation, mirror (when applicable),
    /// and translation baked in — drawing it requires no further transformation aside from
    /// the per-sheet Y offset. The <see cref="Transform"/> field is a separate, rigid-only
    /// transform (rotation + translation) for callers that need to transform the original
    /// Rhino curve directly; mirror is NOT baked into <see cref="Transform"/> and must be
    /// applied separately by the caller when <see cref="IsMirrored"/> is true.
    /// </summary>
    public sealed class PlacementResult
    {
        /// <summary>
        /// Index of this part in the request's <see cref="NestRequest.Polygons"/> list.
        /// Lets the draw layer match a placement back to the user's original Rhino object.
        /// </summary>
        public int OriginalIndex { get; }

        /// <summary>
        /// Zero-based index of the sheet this part was placed on. Used by the draw layer
        /// to compute per-sheet Y offset for vertical stacking.
        /// </summary>
        public int Sheet { get; }

        /// <summary>
        /// Rigid transform (rotation + translation) from the part's original 2D coordinates
        /// to its placed position on the sheet. Does NOT include mirror — when
        /// <see cref="IsMirrored"/> is true, the caller must apply a mirror to the source
        /// curve BEFORE applying this transform.
        ///
        /// The <see cref="Transform2D"/> contract guarantees rigid-only (determinant +1);
        /// keeping mirror separate preserves that contract everywhere downstream.
        /// </summary>
        public Transform2D Transform { get; }

        /// <summary>
        /// Rotation applied to the part, in degrees, CCW positive. Redundant with the
        /// rotation encoded in <see cref="Transform"/>, but exposed directly for diagnostics
        /// and report output.
        /// </summary>
        public double RotationDeg { get; }

        /// <summary>
        /// True if the part was mirrored (X-flipped) before rotation and translation.
        /// The flip is baked into <see cref="PlacedPolygon"/> already, so direct drawing
        /// from <see cref="PlacedPolygon"/> needs no extra mirror step.
        ///
        /// For drawing from the ORIGINAL Rhino curve via <see cref="Transform"/>, the
        /// caller must:
        ///   1. If <see cref="IsMirrored"/>, apply a mirror across the curve's local Y axis
        ///      (or any axis through the origin, then re-rotate — the convention is X-flip
        ///      to match <see cref="Polygon.Mirror"/>).
        ///   2. Apply <see cref="Transform"/>.
        ///
        /// For BLF placements (Phase 1) this is always false.
        /// </summary>
        public bool IsMirrored { get; }

        /// <summary>
        /// The placed 2D polygon: original outline rotated, mirrored (if applicable), and
        /// translated to its final position on the sheet. Already in sheet-local coordinates;
        /// the only further transform needed at draw time is the per-sheet Y offset for
        /// multi-sheet vertical stacking.
        /// </summary>
        public Polygon PlacedPolygon { get; }

        /// <summary>
        /// Phase 2 constructor with mirror flag.
        /// </summary>
        public PlacementResult(
            int originalIndex,
            int sheet,
            Transform2D transform,
            double rotationDeg,
            bool isMirrored,
            Polygon placedPolygon)
        {
            if (originalIndex < 0)
                throw new ArgumentOutOfRangeException(nameof(originalIndex), "Must be non-negative.");
            if (sheet < 0)
                throw new ArgumentOutOfRangeException(nameof(sheet), "Must be non-negative.");
            if (placedPolygon == null)
                throw new ArgumentNullException(nameof(placedPolygon));

            OriginalIndex = originalIndex;
            Sheet = sheet;
            Transform = transform;
            RotationDeg = rotationDeg;
            IsMirrored = isMirrored;
            PlacedPolygon = placedPolygon;
        }

        /// <summary>
        /// Phase 1 compatibility constructor. Defaults <see cref="IsMirrored"/> to false.
        /// Lets the existing BLF engine path keep its current call sites unchanged.
        /// </summary>
        public PlacementResult(
            int originalIndex,
            int sheet,
            Transform2D transform,
            double rotationDeg,
            Polygon placedPolygon)
            : this(originalIndex, sheet, transform, rotationDeg, isMirrored: false, placedPolygon)
        {
        }

        public override string ToString()
        {
            return IsMirrored
                ? $"PlacementResult(part={OriginalIndex}, sheet={Sheet}, rot={RotationDeg:F1}°, mirrored)"
                : $"PlacementResult(part={OriginalIndex}, sheet={Sheet}, rot={RotationDeg:F1}°)";
        }
    }
}