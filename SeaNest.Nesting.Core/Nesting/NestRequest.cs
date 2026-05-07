using System;
using System.Collections.Generic;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Rotation resolution used when searching for valid orientations.
    /// Finer steps give tighter packing at the cost of more candidate tries per part.
    /// </summary>
    public enum RotationStep
    {
        /// <summary>0, 90, 180, 270 degrees. Fastest. Default.</summary>
        Every90,
        /// <summary>0, 45, 90, ... 315 degrees.</summary>
        Every45,
        /// <summary>0, 15, 30, ... 345 degrees.</summary>
        Every15,
        /// <summary>0, 5, 10, ... 355 degrees. Slowest; best packing.</summary>
        Every5Slow
    }

    /// <summary>
    /// Nesting algorithm selection. Determines which engine path runs in
    /// <see cref="NestingEngine.Nest"/>.
    /// </summary>
    public enum NestingAlgorithm
    {
        /// <summary>
        /// Phase 1 Bottom-Left-Fill with bbox-corner candidates and slide-tightening.
        /// Fastest. Typical 60–75% utilization on irregular boat parts. Use for
        /// quick previews and as a regression baseline.
        /// </summary>
        BLF,

        /// <summary>
        /// Phase 2 No-Fit-Polygon greedy placement, single pass. Walks NFP edges
        /// for candidate positions instead of bbox corners. Typical 78–85% with
        /// mirror enabled. Use for normal production runs.
        /// </summary>
        NFP_Greedy,

        /// <summary>
        /// Phase 2 NFP greedy wrapped in simulated annealing. Runs the greedy
        /// engine many times with perturbed orderings, accepting worse solutions
        /// probabilistically to escape local minima. Typical 85–92% with mirror
        /// enabled. Use when sheet count matters and you can afford the time.
        /// Honors <see cref="TimeBudget"/>.
        /// </summary>
        NFP_Annealed
    }

    /// <summary>
    /// Input to the nesting engine. Immutable.
    ///
    /// All lengths (sheet dimensions, margin, spacing, polygon coordinates) must be in
    /// the same unit. The caller is responsible for any Rhino unit-system conversion
    /// before constructing this request.
    /// </summary>
    public sealed class NestRequest
    {
        /// <summary>Polygons to nest, in input order. Each is a flat 2D outline.</summary>
        public IReadOnlyList<Polygon> Polygons { get; }

        /// <summary>Sheet width (long axis conventionally, but engine does not care).</summary>
        public double SheetWidth { get; }

        /// <summary>Sheet height.</summary>
        public double SheetHeight { get; }

        /// <summary>Sheet thickness (informational only; used for labels).</summary>
        public double SheetThickness { get; }

        /// <summary>Inset from every sheet edge. Usable area is (W - 2*margin) by (H - 2*margin).</summary>
        public double Margin { get; }

        /// <summary>Minimum clearance between adjacent parts.</summary>
        public double Spacing { get; }

        /// <summary>Angular resolution for rotation search.</summary>
        public RotationStep RotationStep { get; }

        /// <summary>
        /// Which algorithm path to run. Default is <see cref="NestingAlgorithm.BLF"/>
        /// for compatibility with Phase 1 callers; new callers should request
        /// <see cref="NestingAlgorithm.NFP_Greedy"/> or <see cref="NestingAlgorithm.NFP_Annealed"/>.
        /// </summary>
        public NestingAlgorithm Algorithm { get; }

        /// <summary>
        /// When true, the NFP engine considers mirrored (X-flipped) variants of each
        /// part as additional orientations. Doubles the orientation search space and
        /// the NFP cache size, but typically gains 5–10 percentage points of utilization
        /// on parts with manual-flip-friendly silhouettes.
        ///
        /// Ignored for <see cref="NestingAlgorithm.BLF"/> (Phase 1 path does not flip).
        /// Default is true: mirror is the right choice for sheet-metal boat parts where
        /// either side can face up.
        /// </summary>
        public bool AllowMirror { get; }

        /// <summary>
        /// Wall-clock time budget for <see cref="NestingAlgorithm.NFP_Annealed"/>.
        /// Annealing checks the budget between iterations and returns its best result
        /// so far when exceeded. Ignored for BLF and NFP_Greedy.
        ///
        /// Default 30 seconds — long enough for ~50–150 iterations on typical jobs,
        /// short enough not to surprise interactive users.
        /// </summary>
        public TimeSpan TimeBudget { get; }

        /// <summary>
        /// Phase 2 constructor. Use this for new code.
        /// </summary>
        public NestRequest(
            IReadOnlyList<Polygon> polygons,
            double sheetWidth,
            double sheetHeight,
            double sheetThickness,
            double margin,
            double spacing,
            RotationStep rotationStep,
            NestingAlgorithm algorithm,
            bool allowMirror,
            TimeSpan timeBudget)
        {
            if (polygons == null) throw new ArgumentNullException(nameof(polygons));
            if (polygons.Count == 0) throw new ArgumentException("At least one polygon required.", nameof(polygons));
            if (sheetWidth <= 0) throw new ArgumentException("Sheet width must be positive.", nameof(sheetWidth));
            if (sheetHeight <= 0) throw new ArgumentException("Sheet height must be positive.", nameof(sheetHeight));
            if (sheetThickness < 0) throw new ArgumentException("Sheet thickness cannot be negative.", nameof(sheetThickness));
            if (margin < 0) throw new ArgumentException("Margin cannot be negative.", nameof(margin));
            if (spacing < 0) throw new ArgumentException("Spacing cannot be negative.", nameof(spacing));
            if (margin * 2 >= sheetWidth || margin * 2 >= sheetHeight)
                throw new ArgumentException("Margin consumes the entire sheet; no usable area remains.", nameof(margin));
            if (timeBudget < TimeSpan.Zero)
                throw new ArgumentException("Time budget cannot be negative.", nameof(timeBudget));

            Polygons = polygons;
            SheetWidth = sheetWidth;
            SheetHeight = sheetHeight;
            SheetThickness = sheetThickness;
            Margin = margin;
            Spacing = spacing;
            RotationStep = rotationStep;
            Algorithm = algorithm;
            AllowMirror = allowMirror;
            TimeBudget = timeBudget;
        }

        /// <summary>
        /// Phase 1 compatibility constructor. Existing callers continue to compile and run BLF
        /// exactly as before. Defaults: <see cref="NestingAlgorithm.BLF"/>, mirror disabled
        /// (BLF doesn't use it anyway), 30-second time budget (irrelevant for BLF).
        /// </summary>
        public NestRequest(
            IReadOnlyList<Polygon> polygons,
            double sheetWidth,
            double sheetHeight,
            double sheetThickness,
            double margin,
            double spacing,
            RotationStep rotationStep)
            : this(
                polygons,
                sheetWidth,
                sheetHeight,
                sheetThickness,
                margin,
                spacing,
                rotationStep,
                NestingAlgorithm.BLF,
                allowMirror: false,
                timeBudget: TimeSpan.FromSeconds(30))
        {
        }

        /// <summary>Degrees per rotation step, derived from <see cref="RotationStep"/>.</summary>
        public double RotationStepDegrees
        {
            get
            {
                switch (RotationStep)
                {
                    case RotationStep.Every90: return 90.0;
                    case RotationStep.Every45: return 45.0;
                    case RotationStep.Every15: return 15.0;
                    case RotationStep.Every5Slow: return 5.0;
                    default: return 90.0;
                }
            }
        }

        /// <summary>Number of rotation candidates per part (360 / step).</summary>
        public int RotationCount => (int)(360.0 / RotationStepDegrees);

        /// <summary>Usable width after margin inset on both sides.</summary>
        public double UsableWidth => SheetWidth - 2 * Margin;

        /// <summary>Usable height after margin inset on both sides.</summary>
        public double UsableHeight => SheetHeight - 2 * Margin;
    }
}