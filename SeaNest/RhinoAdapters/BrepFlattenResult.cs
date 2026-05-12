using System;
using System.Collections.Generic;
using Rhino.Geometry;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.RhinoAdapters
{
    /// <summary>
    /// Result of flattening a Brep for nesting: the outer 2D <see cref="Polygon"/>
    /// the engine sees, the matching native Rhino curve for high-fidelity
    /// rendering, plus any inner-loop cut curves (pipe holes, hatch cutouts) that
    /// ride along untouched into the rendered nest.
    ///
    /// Inner loops bypass the nesting engine entirely — placement, NFP, BLF,
    /// overlap checks and FinalVerifier all operate on the outer polygon only. The
    /// orchestrator carries the inner loops alongside in a parallel per-part table
    /// keyed by polygon (= brep) index, and at draw time applies the same
    /// translate+rotate+mirror transform the placed outer received via
    /// <see cref="PolygonToCurve.ToCurveFromOriginal"/>. Parts still nest as solid
    /// envelopes from the algorithm's perspective; we just don't drop the holes
    /// from the cut output.
    ///
    /// Phase 9a adds the parallel <see cref="OuterCurve"/> native form for the
    /// outer outline. The engine continues to consume <see cref="OuterPolygon"/>
    /// (heavily simplified, fast NFP); the renderer prefers <see cref="OuterCurve"/>
    /// when available to emit smooth native geometry instead of the engine's
    /// faceted polygon.
    /// </summary>
    public sealed class BrepFlattenResult
    {
        /// <summary>
        /// Outer boundary polygon as the engine sees it (post-simplification).
        /// This is what flows into <see cref="SeaNest.Nesting.Core.Nesting.NestRequest.Polygons"/>
        /// and becomes <see cref="SeaNest.Nesting.Core.Nesting.PlacementResult.PlacedPolygon"/>
        /// after placement. Vertex count is capped by BrepFlattener's simplification
        /// pass (typically a few hundred); the renderer can use <see cref="OuterCurve"/>
        /// to bypass the polygonal facets when smoother output is desired.
        /// </summary>
        public Polygon OuterPolygon { get; }

        /// <summary>
        /// Native Rhino curve corresponding to <see cref="OuterPolygon"/>, in the
        /// same world-XY plane-local frame (Z=0), produced by routing the brep loop's
        /// native curve through <see cref="BrepFlattener.ProjectCurveToPlaneSpace"/>.
        /// Preserves the original curve's subclass (<see cref="NurbsCurve"/>,
        /// <see cref="PolyCurve"/>, <see cref="ArcCurve"/>, etc.), so smooth arcs
        /// stay smooth and the DXF output emits native SPLINE/ARC/CIRCLE entities
        /// instead of polylines.
        ///
        /// <b>Nullable.</b> Null when the Squish fallback path produces the
        /// polygon — meshed-and-squished surfaces have no native source curve to
        /// recover. The renderer falls back to drawing
        /// <see cref="SeaNest.Nesting.Core.Nesting.PlacementResult.PlacedPolygon"/>
        /// directly in that case.
        ///
        /// <b>Bbox-discrepancy caveat.</b> The native curve's bounding box is
        /// generally a superset of <see cref="OuterPolygon"/>'s, by up to
        /// <c>discretizeTol + simplificationTol</c> (≤ 0.005" typical, ≤ 0.04" worst
        /// case after escalation). On mirrored placements,
        /// <see cref="PolygonToCurve.ToCurveFromOriginal"/> mirrors about the polygon's
        /// bbox-center X — the placed native curve can therefore extend that same
        /// amount beyond where the polygon-based math expects. Below the typical
        /// 0.25" spacing parameter by 6×; invisible at cut tolerances. Mitigation if
        /// it ever bites is to tighten BrepFlattener's simplification tolerance.
        /// </summary>
        public Curve OuterCurve { get; }

        /// <summary>
        /// Inner-loop cut curves in the same world-XY coordinate frame as
        /// <see cref="OuterPolygon"/>. Native curve subclasses (closed PolylineCurve,
        /// ArcCurve, NurbsCurve …) at Z=0. Empty list if the part has no holes;
        /// never null.
        /// </summary>
        public IReadOnlyList<Curve> InnerLoops { get; }

        /// <summary>
        /// Phase 19b — structural reference scribe lines for the plate, in the
        /// same world-XY plane-local frame as <see cref="OuterPolygon"/>. These
        /// are the curves where structural members (frames, longitudinals,
        /// bulkheads) intersect the plate's cut face; they ride along with the
        /// outer through placement (same <see cref="PolygonToCurve.ToCurveFromOriginal"/>
        /// transform as <see cref="InnerLoops"/>) and emit on a dedicated
        /// <c>SeaNest_Scribe</c> layer for low-power-marking / etch-tool routing
        /// at the plate cutter. Empty list if the user didn't supply scribe
        /// members or no intersections were found; never null.
        ///
        /// Currently populated for Step 1 (face-to-polygon) and Step 2
        /// (fully-planar Brep) flatten paths only. Step 3 (Unroll) and Step 4
        /// (Squish) emit a one-shot warning via
        /// <see cref="BrepFlattener.ScribeWarning"/> and drop the scribes for
        /// the affected part. Curved-plate (Unroll) support is deferred to
        /// Phase 19b.2 via <c>Unroller.AddFollowingGeometry</c>.
        ///
        /// Phase 19b.0.3: intersection curves are projected to the flatten
        /// plane unconditionally rather than filtered for plane-coplanarity.
        /// Plate-vs-frame intersection curves naturally sit on the plate's
        /// physical faces (~half a thickness off the twin-pair average plane);
        /// orthographic projection lands the correct 2D scribe regardless of
        /// source face. A post-projection length filter drops side-strip
        /// intersections that project to near-point artifacts.
        /// </summary>
        public IReadOnlyList<Curve> ScribeLines { get; }

        public BrepFlattenResult(
            Polygon outerPolygon,
            Curve outerCurve,
            IReadOnlyList<Curve> innerLoops,
            IReadOnlyList<Curve> scribeLines = null)
        {
            OuterPolygon = outerPolygon ?? throw new ArgumentNullException(nameof(outerPolygon));
            OuterCurve = outerCurve;   // may be null — Squish path has no native source
            InnerLoops = innerLoops ?? Array.Empty<Curve>();
            ScribeLines = scribeLines ?? Array.Empty<Curve>();
        }
    }
}
