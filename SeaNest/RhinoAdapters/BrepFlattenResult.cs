using System;
using System.Collections.Generic;
using Rhino.Geometry;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.RhinoAdapters
{
    /// <summary>
    /// Result of flattening a Brep for nesting: the outer 2D <see cref="Polygon"/>
    /// the engine sees, plus any inner-loop cut curves (pipe holes, hatch cutouts)
    /// that ride along untouched into the rendered nest.
    ///
    /// Inner loops bypass the nesting engine entirely — placement, NFP, BLF,
    /// overlap checks and FinalVerifier all operate on the outer ring only. The
    /// orchestrator carries the inner loops alongside in a parallel per-part table
    /// keyed by polygon (= brep) index, and at draw time applies the same
    /// translate+rotate+mirror transform the placed outer received via
    /// <see cref="PolygonToCurve.ToCurveFromOriginal"/>. Parts still nest as solid
    /// envelopes from the algorithm's perspective; we just don't drop the holes
    /// from the cut output.
    /// </summary>
    public sealed class BrepFlattenResult
    {
        /// <summary>Outer boundary polygon as the engine sees it (post-simplification).</summary>
        public Polygon Outer { get; }

        /// <summary>
        /// Inner-loop cut curves in the same world-XY coordinate frame as
        /// <see cref="Outer"/>. Closed PolylineCurves at Z=0. Empty list if the
        /// part has no holes; never null.
        /// </summary>
        public IReadOnlyList<Curve> InnerLoops { get; }

        public BrepFlattenResult(Polygon outer, IReadOnlyList<Curve> innerLoops)
        {
            Outer = outer ?? throw new ArgumentNullException(nameof(outer));
            InnerLoops = innerLoops ?? Array.Empty<Curve>();
        }
    }
}
