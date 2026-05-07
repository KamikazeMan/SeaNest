using System;
using System.Collections.Generic;
using System.Linq;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Result of a nesting run. Immutable.
    ///
    /// <see cref="Placements"/> holds every successfully placed part (in original input order,
    /// not nesting order). <see cref="UnplacedIndices"/> holds indices of parts that could not
    /// fit on any sheet at any rotation — typically because the part is larger than the sheet.
    /// </summary>
    public sealed class NestResponse
    {
        /// <summary>All successful placements, sorted by original input index.</summary>
        public IReadOnlyList<PlacementResult> Placements { get; }

        /// <summary>Indices (in the original input list) of parts that could not be placed.</summary>
        public IReadOnlyList<int> UnplacedIndices { get; }

        /// <summary>Number of sheets used. Zero if nothing was placed.</summary>
        public int SheetCount { get; }

        /// <summary>Total area of all placed parts (sum of absolute polygon areas).</summary>
        public double TotalPlacedArea { get; }

        /// <summary>Total usable sheet area across all sheets used
        /// (SheetCount * usableWidth * usableHeight).</summary>
        public double TotalUsableSheetArea { get; }

        /// <summary>Material utilization as a fraction in [0, 1]: placed area divided by usable sheet area.
        /// Returns 0 if no sheets were used.</summary>
        public double Utilization =>
            TotalUsableSheetArea > 0 ? TotalPlacedArea / TotalUsableSheetArea : 0.0;

        /// <summary>Wall-clock time the engine spent producing this result.</summary>
        public TimeSpan ElapsedTime { get; }

        public NestResponse(
            IReadOnlyList<PlacementResult> placements,
            IReadOnlyList<int> unplacedIndices,
            double totalPlacedArea,
            double totalUsableSheetArea,
            TimeSpan elapsedTime)
        {
            if (placements == null) throw new ArgumentNullException(nameof(placements));
            if (unplacedIndices == null) throw new ArgumentNullException(nameof(unplacedIndices));

            Placements = placements;
            UnplacedIndices = unplacedIndices;
            SheetCount = placements.Count == 0 ? 0 : placements.Max(p => p.Sheet) + 1;
            TotalPlacedArea = totalPlacedArea;
            TotalUsableSheetArea = totalUsableSheetArea;
            ElapsedTime = elapsedTime;
        }

        public override string ToString()
            => $"NestResponse(Placed={Placements.Count}, Unplaced={UnplacedIndices.Count}, Sheets={SheetCount}, Util={Utilization:P1}, {ElapsedTime.TotalSeconds:F2}s)";
    }
}