using System;
using System.Collections.Generic;
using System.Linq;
using SeaNest.Nesting.Core.Nesting;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Verification
{
    /// <summary>
    /// Result of running <see cref="FinalVerifier"/> over a set of placements.
    /// </summary>
    public sealed class VerificationResult
    {
        /// <summary>True if no overlaps were found.</summary>
        public bool IsValid => OverlappingPartCount == 0;

        /// <summary>Distinct parts (by original index) involved in at least one overlap.</summary>
        public int OverlappingPartCount { get; }

        /// <summary>Original indices of parts involved in at least one overlap.</summary>
        public IReadOnlyList<int> OverlappingOriginalIndices { get; }

        public VerificationResult(IReadOnlyList<int> overlappingOriginalIndices)
        {
            OverlappingOriginalIndices = overlappingOriginalIndices ?? Array.Empty<int>();
            OverlappingPartCount = OverlappingOriginalIndices.Count;
        }
    }

    /// <summary>
    /// Final overlap check before the engine's results are written to the Rhino document.
    ///
    /// Stricter than the engine's internal checks: no spacing inflation, tight numerical tolerance.
    /// If this fails it indicates a bug in the engine — the caller should abort the entire nest
    /// and surface the error to the user rather than drawing potentially overlapping parts.
    ///
    /// Only checks pairs of parts on the SAME sheet; parts on different sheets cannot overlap
    /// because they are drawn with per-sheet Y offsets.
    /// </summary>
    public static class FinalVerifier
    {
        /// <summary>Tight numerical tolerance for verification. Intersections smaller than
        /// tolerance² by area are treated as floating-point noise, not real overlap.</summary>
        public const double VerifyTolerance = 1e-4;

        /// <summary>
        /// Verify that no two placements on the same sheet overlap.
        /// </summary>
        public static VerificationResult Verify(IReadOnlyList<PlacementResult> placements)
        {
            if (placements == null) throw new ArgumentNullException(nameof(placements));

            if (placements.Count < 2)
                return new VerificationResult(Array.Empty<int>());

            // Group placements by sheet index so we only pairwise-check within a sheet.
            var bySheet = placements
                .GroupBy(p => p.Sheet)
                .ToList();

            var involved = new HashSet<int>();

            foreach (var sheetGroup in bySheet)
            {
                var sheetPlacements = sheetGroup.ToList();
                int n = sheetPlacements.Count;

                for (int i = 0; i < n - 1; i++)
                {
                    var a = sheetPlacements[i];
                    for (int j = i + 1; j < n; j++)
                    {
                        var b = sheetPlacements[j];
                        if (OverlapChecker.Overlaps(a.PlacedPolygon, b.PlacedPolygon, VerifyTolerance))
                        {
                            involved.Add(a.OriginalIndex);
                            involved.Add(b.OriginalIndex);
                        }
                    }
                }
            }

            return new VerificationResult(involved.OrderBy(i => i).ToList());
        }
    }
}