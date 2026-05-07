using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Inner-Fit Polygon — the set of valid translations for an oriented part such that
    /// the whole part stays inside the sheet. The placement engine intersects this with
    /// the complement of the union of placed-part NFPs to get the feasible placement
    /// region for a single sheet.
    ///
    /// For a rectangular sheet (the only sheet type Phase 2 supports), the IFP is itself
    /// a rectangle. With our reference-point convention (<see cref="OrientedPart.CanonicalPolygon"/>
    /// has bbox-min at the origin), the IFP is the sheet's usable rectangle inset by the
    /// part's width on the right and the part's height on the top:
    ///
    ///   IFP = [margin, margin + (usableW - partW)] × [margin, margin + (usableH - partH)]
    ///
    /// If <c>partW &gt; usableW</c> or <c>partH &gt; usableH</c>, the part cannot fit on
    /// the sheet in this orientation and the IFP is empty (returned as null).
    ///
    /// This class is rectangular-sheet-only by design. Phase 3 (offcut re-feeding) will
    /// replace it with a Minkowski-difference-based IFP using Clipper2; the public surface
    /// here is shaped to make that swap a one-method change.
    /// </summary>
    public static class InnerFitPolygon
    {
        /// <summary>
        /// Compute the IFP rectangle for a single oriented part on a rectangular sheet.
        /// Returns null if the part is too large to fit on the sheet in this orientation.
        ///
        /// All inputs in model units. Sheet origin is (0, 0); usable area starts at
        /// (<paramref name="margin"/>, <paramref name="margin"/>) and extends to
        /// (<paramref name="sheetWidth"/> - <paramref name="margin"/>,
        ///  <paramref name="sheetHeight"/> - <paramref name="margin"/>).
        ///
        /// Returned rectangle is in TRANSLATION space — i.e., the corners are valid
        /// (dx, dy) values for placing the part's bbox-min reference point.
        /// </summary>
        /// <param name="part">Oriented part. Bbox-min is assumed to be at (0, 0).</param>
        /// <param name="sheetWidth">Full sheet width.</param>
        /// <param name="sheetHeight">Full sheet height.</param>
        /// <param name="margin">Inset from each sheet edge.</param>
        public static BoundingBox2D? Compute(
            OrientedPart part,
            double sheetWidth,
            double sheetHeight,
            double margin)
        {
            // Part width and height in this orientation. By the OrientedPart contract,
            // CanonicalPolygon's bbox-min is at the origin, so MaxX/MaxY ARE the dimensions.
            double partW = part.BBox.MaxX;
            double partH = part.BBox.MaxY;

            double usableW = sheetWidth - 2.0 * margin;
            double usableH = sheetHeight - 2.0 * margin;

            // Tolerance for the fit check. A part that's nominally exactly equal to the
            // usable dimension can fail a strict ">" comparison after floating-point drift
            // through rotation; a tiny epsilon prevents spurious "doesn't fit" reports for
            // parts cut to nominal sheet size.
            const double fitEps = 1e-9;
            if (partW > usableW + fitEps || partH > usableH + fitEps)
                return null;

            // Translation-space rectangle. The reference point can sit anywhere in
            // [margin, margin + (usableW - partW)] × [margin, margin + (usableH - partH)]
            // and the part will lie entirely within the usable area.
            double minX = margin;
            double minY = margin;
            double maxX = margin + (usableW - partW);
            double maxY = margin + (usableH - partH);

            // Clamp tiny negatives produced by the eps allowance above.
            if (maxX < minX) maxX = minX;
            if (maxY < minY) maxY = minY;

            return new BoundingBox2D(minX, minY, maxX, maxY);
        }

        /// <summary>
        /// Quick fit test without computing the full IFP. Returns true if the oriented
        /// part can fit on a rectangular sheet at all in this orientation.
        ///
        /// Equivalent to <c>Compute(...).HasValue</c> but skips the rectangle allocation.
        /// Used by the placement engine's orientation enumerator to skip orientations that
        /// can't ever produce a valid placement, before doing expensive NFP work.
        /// </summary>
        public static bool Fits(
            OrientedPart part,
            double sheetWidth,
            double sheetHeight,
            double margin)
        {
            const double fitEps = 1e-9;
            double usableW = sheetWidth - 2.0 * margin;
            double usableH = sheetHeight - 2.0 * margin;
            return part.BBox.MaxX <= usableW + fitEps
                && part.BBox.MaxY <= usableH + fitEps;
        }
    }
}