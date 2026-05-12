using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Geometry;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.RhinoAdapters
{
    /// <summary>
    /// Converts a Rhino <see cref="Brep"/> into a 2D <see cref="Polygon"/> for nesting.
    ///
    /// Dispatch order (first success wins):
    ///   1. Thickened-plate Breps: pick the largest coplanar-face region and use its outline,
    ///      projected onto the face's own plane (handles vertical/tilted parts).
    ///   2. Fully flat Breps: outer boundary on the shared face plane.
    ///   3. Developable curved Breps: unroll via Rhino's Unroller.
    ///   4. Fallback — Squish the largest face regardless of planarity. Handles freeform Patch
    ///      surfaces and near-flat surfaces that fooled IsPlanar but didn't produce a usable
    ///      outline from Step 1.
    /// </summary>
    public static class BrepFlattener
    {
        private const double UnrollToleranceFactor = 0.1;
        private const double UnrollAngleToleranceDegrees = 0.5;
        private const double DiscretizationToleranceFactor = 0.1;

        /// <summary>
        /// Pre-NFP simplification tolerance for the flattened polygon, in model
        /// units (inches). 0.005" sits comfortably below typical CAM cut
        /// precision (0.005-0.010" for plate cutting). Curve.ToPolyline emits
        /// vertices every 0.5° of curvature regardless of CAM relevance,
        /// producing 500-3000+ vertex polygons on real boat parts; DP at this
        /// tolerance collapses arc densification back to feature-only vertices
        /// without visible shape change.
        /// </summary>
        private const double SimplifyTolerance = 0.005;

        /// <summary>
        /// Vertex-count cap. Polygons exceeding this after the initial DP pass
        /// trigger tolerance escalation. 500 is well above the natural vertex
        /// count of any sensibly-meshed boat part (usually under 100 once
        /// arcs collapse to chords); a polygon hitting this is over-tessellated
        /// and would otherwise dominate MinkowskiDiff cost.
        /// </summary>
        private const int MaxVertices = 500;

        /// <summary>Each escalation doubles the simplification tolerance.</summary>
        private const double EscalationFactor = 2.0;

        /// <summary>
        /// Hard ceiling on tolerance escalation. With <see cref="SimplifyTolerance"/>
        /// = 0.005 and <see cref="EscalationFactor"/> = 2.0, three escalations cap
        /// at 0.04" — still below most CAM reject thresholds but well into
        /// "warn the user, this part needs attention" territory.
        /// </summary>
        private const int MaxEscalations = 3;

        /// <summary>
        /// Multiplier on the document tolerance for the planarity-related checks
        /// in Step 1 (face selection AND face-plane extraction inside
        /// <see cref="FaceToPolygon"/>). Phase 14.1: boat plate faces are flat
        /// to ~0.01–0.02" on smaller parts but not to the strict modelTol of
        /// ~0.001"; Phase 14.1.1: 186"-class boat parts have plate faces flat
        /// to ~0.05–0.08" (non-planarity scales with span), and the
        /// parameterless <c>BrepFace.TryGetPlane</c> in <see cref="FaceToPolygon"/>
        /// was rejecting them with its own RhinoMath.ZeroTolerance default —
        /// same parameterless-overload bug Phase 7c.3 fixed in
        /// <see cref="FindLargestPlanarFace"/>. Both call sites now use this
        /// loosened tolerance.
        /// <para>
        /// 100× modelTol = 0.10" for inch docs at default modelTol = 0.001".
        /// Catches plate faces that deviate up to 0.10" while genuinely curved
        /// faces (developable surfaces with mm-scale curvature, cones, sphere
        /// segments) still fail. The bound matches typical plate-cutter kerf
        /// width (0.05–0.10"), so any geometric error introduced by best-fit-
        /// plane projection is within what the cutter would absorb anyway.
        /// </para>
        /// <para>
        /// Only the two planarity checks use this loosened tolerance — polyline
        /// discretization (<see cref="CurveToPolygonOnPlane"/>) and plane-to-
        /// plane projection (<see cref="ProjectCurveToPlaneSpace"/>) continue
        /// to operate at modelTol so the polygon vertices and native curves
        /// stay accurate to model tolerance on whichever plane was chosen.
        /// If a future precision-machining case needs tighter detection, drop
        /// this to ~25× or ~10×.
        /// </para>
        /// </summary>
        private const double PlanarityToleranceFactor = 100.0;

        /// <summary>
        /// Phase 14.1 area-floor for plate-face selection, relative to the
        /// largest qualifying-planar face. A face must contribute at least
        /// this fraction of the maximum candidate area to compete; smaller
        /// faces are rejected.
        /// <para>
        /// Edge strips on thick plates are 0.1–0.8% of the plate face's area
        /// (e.g. 7.5 / 6000 ≈ 0.1%), well below this 10% threshold. A stepped
        /// or two-tiered plate where one face is &lt;10% of the other will
        /// have the smaller face filtered out — correct for plate selection,
        /// since the largest face is the one we want to project the outline
        /// from. Two roughly-equal plate faces (e.g. top and bottom of a
        /// uniform slab) both pass the floor; largest wins by area as before.
        /// </para>
        /// </summary>
        private const double FaceAreaFloorRelative = 0.10;

        /// <summary>
        /// Phase 14.1.2 twin-face plate detection: maximum angle between a
        /// face-pair's normals to count as "anti-parallel," expressed as the
        /// cosine of the threshold for direct comparison against the unit-
        /// normal dot product. cos(10°) ≈ 0.9848 → two faces are anti-parallel
        /// within 10° iff <c>dot(n_i, n_j) ≤ -0.9848</c>. 10° gives enough
        /// slack for moderate plate-face curvature (each face's center-point
        /// normal tilted up to 5° from the "average" direction) while still
        /// firmly rejecting perpendicular faces, plate edge-strips, etc.
        /// </summary>
        private const double TwinAntiparallelCosThreshold = 0.9848;   // cos(10°)

        /// <summary>
        /// Phase 14.1.2 twin-face plate detection: minimum ratio of smaller-
        /// face-area to larger-face-area for a pair to qualify as a plate
        /// twin. Plate top/bottom should be near-equal in area; allowing
        /// down to 0.7 tolerates slight curvature/trim differences but
        /// firmly rejects an edge strip (area ratio ~0.01) being mis-paired
        /// with a plate face.
        /// </summary>
        private const double TwinAreaRatioMin = 0.7;

        /// <summary>
        /// Phase 14.1.2 twin-face plate detection: maximum ratio of
        /// perpendicular face-to-face separation to the larger of the two
        /// faces' bounding-box diagonals. Plates are wide and thin: a 200"
        /// plate at 0.25" thick has ratio ≈ 0.001; a 6"-cube has ratio
        /// ≈ 0.16, so the cube's "twin" face pairs correctly fail this test.
        /// 0.1 is the rough boundary between "thin enough to be a plate"
        /// and "too thick to be a plate."
        /// </summary>
        private const double TwinSeparationRatioMax = 0.1;

        /// <summary>
        /// Callback invoked when a part was flattened using Squish. Used by the command layer
        /// to inform the user that dimensions may be approximate for curved parts.
        /// </summary>
        public static Action<string> SquishWarning { get; set; }

        /// <summary>
        /// Callback invoked when a part's polygon was simplified at an escalated
        /// tolerance because the initial pass left it above the vertex cap.
        /// Surfaces over-tessellated input geometry to the user without aborting
        /// the nest. Wired by the command layer to RhinoApp.WriteLine.
        /// </summary>
        public static Action<string> SimplifyWarning { get; set; }

        public static BrepFlattenResult Flatten(Brep brep, RhinoDoc doc)
        {
            var raw = FlattenRaw(brep, doc, out var outerCurve, out var innerLoops);
            if (raw == null) return null;

            // Pre-NFP simplification: collapse arc densification from
            // Curve.ToPolyline / SquishMesh down to feature-only vertices.
            // The Brep itself is not modified — this only affects the Polygon
            // used downstream by the nesting engine.
            int rawCount = raw.Count;
            var simplified = raw.SimplifyToTarget(
                SimplifyTolerance, MaxVertices, EscalationFactor, MaxEscalations,
                out double finalTol);

            if (finalTol > SimplifyTolerance + 1e-12)
            {
                SimplifyWarning?.Invoke(
                    $"Part simplified from {rawCount} to {simplified.Count} vertices " +
                    $"at tolerance {finalTol:G3}\" (escalated from {SimplifyTolerance:G3}\" " +
                    $"because raw exceeded {MaxVertices}-vertex cap). Consider remeshing the source " +
                    $"Brep at a coarser angle tolerance if this fires often.");
            }

            // Outer native curve and inner loops are NOT simplified — fidelity
            // matters for cut output, they don't enter NFP, and the renderer
            // consumes them directly.
            return new BrepFlattenResult(simplified, outerCurve, innerLoops);
        }

        private static Polygon FlattenRaw(Brep brep, RhinoDoc doc, out Curve outerCurve, out List<Curve> innerLoops)
        {
            outerCurve = null;
            innerLoops = new List<Curve>();

            if (brep == null) throw new ArgumentNullException(nameof(brep));
            if (doc == null) throw new ArgumentNullException(nameof(doc));

            double modelTol = doc.ModelAbsoluteTolerance;
            double discretizeTol = modelTol * DiscretizationToleranceFactor;
            double angleTolRad = RhinoMath.ToRadians(UnrollAngleToleranceDegrees);

            // Step 1: Thickened plate — twin-face topology first, then fall
            // back to largest-planar-face selection. Phase 14.1.2: twin-pair
            // detection identifies plates by structure (anti-parallel similar-
            // area faces with small separation) and handles slightly-curved
            // plate faces that fail any practical planarity tolerance. Falls
            // back to FindLargestPlanarFace for single-surface inputs, non-
            // plate solids, and other non-twin cases.
            BrepFace plateFace;
            Plane? projectionPlane = null;
            var twinResult = FindTwinPlateFace(brep, modelTol);
            if (twinResult != null)
            {
                plateFace = twinResult.Value.Face;
                projectionPlane = twinResult.Value.Plane;
            }
            else
            {
                plateFace = FindLargestPlanarFace(brep, modelTol);
            }
            if (plateFace != null)
            {
                var poly = FaceToPolygon(plateFace, discretizeTol, angleTolRad, modelTol, modelTol * PlanarityToleranceFactor, projectionPlane, out outerCurve, innerLoops);
                if (poly != null) return poly;
                outerCurve = null;
                innerLoops.Clear();
            }

            // Step 2: Fully flat open Brep.
            if (IsFullyPlanar(brep, modelTol))
            {
                Plane plane;
                if (TryGetBrepPlane(brep, modelTol, out plane))
                {
                    var poly = FlatBrepToPolygonOnPlane(brep, plane, modelTol, discretizeTol, angleTolRad, modelTol, out outerCurve, innerLoops);
                    if (poly != null) return poly;
                    outerCurve = null;
                    innerLoops.Clear();
                }
            }

            // Step 3: Developable curved Brep — unroll.
            double unrollTol = modelTol * UnrollToleranceFactor;
            var unrolled = Unroll(brep, unrollTol, angleTolRad);
            if (unrolled != null)
            {
                var poly = FlatBrepToPolygonOnPlane(unrolled, Plane.WorldXY, modelTol, discretizeTol, angleTolRad, modelTol, out outerCurve, innerLoops);
                if (poly != null) return poly;
                outerCurve = null;
                innerLoops.Clear();
            }

            // Step 4: Fallback — squish the largest face regardless of planarity.
            // No native source curve exists for a meshed-and-squished surface; the
            // renderer will fall back to drawing the placed polygon directly.
            var squishFace = FindLargestFace(brep);
            if (squishFace != null)
            {
                var poly = SquishFaceToPolygon(squishFace, discretizeTol, angleTolRad, out outerCurve, innerLoops);
                if (poly != null)
                {
                    SquishWarning?.Invoke(
                        "Squish flattening used — dimensions may be approximate for curved parts.");
                    return poly;
                }
                outerCurve = null;
                innerLoops.Clear();
            }

            return null;
        }

        // ------------------------------------------------------------------
        // Step 1 support: largest coplanar-face region
        // ------------------------------------------------------------------

        private static BrepFace FindLargestPlanarFace(Brep brep, double tolerance)
        {
            // Orientation-agnostic plate detection: largest planar face wins, full
            // stop. Coplanarity grouping and the world-Z tiebreaker that the
            // pre-7c.3 version used both biased toward "lying flat on world XY"
            // and broke for orientation-arbitrary inputs (notably single-face
            // NURBS Breps extracted from solids, where the parameterless
            // TryGetPlane overload would silently fail at tight default
            // tolerance even after IsPlanar(modelTol) succeeded). The explicit-
            // tolerance TryGetPlane below matches Step 2's TryGetBrepPlane and
            // closes that gap. Top/bottom-of-uniform-plate handedness ambiguity
            // is acceptable: outer and inner loops travel through the same
            // facePlane downstream, so cut output is geometrically invariant
            // for symmetric stock.
            // Phase 14.1: planarity check uses a loosened tolerance so that
            // slightly-curved plate faces (e.g. boat plates flat to 0.01–0.02"
            // but not to the strict modelTol) register as planar. The
            // `tolerance` parameter passed in by the caller remains the
            // strict tolerance for the rest of the flatten pipeline (joins,
            // discretization, downstream simplification); only IsPlanar and
            // TryGetPlane below use the loosened value.
            double planarityTol = tolerance * PlanarityToleranceFactor;

            BrepFace best = null;
            double bestArea = -1;

            // Phase 14.1 first pass: collect every face that survives the
            // (loosened) planarity, plane-fit, duplicate-face, and area-mass
            // checks. We need all of them before applying the relative
            // area-floor in the second pass.
            var candidates = new List<(BrepFace face, double area)>();

            foreach (var face in brep.Faces)
            {
                if (!face.IsPlanar(planarityTol)) continue;
                Plane plane;
                if (!face.TryGetPlane(out plane, planarityTol)) continue;

                var faceBrep = face.DuplicateFace(false);
                if (faceBrep == null) continue;
                var amp = AreaMassProperties.Compute(faceBrep);
                if (amp == null) continue;

                candidates.Add((face, amp.Area));
            }

            // Phase 14.1 second pass: apply the area-floor relative to the
            // max candidate area, then pick the largest remaining.
            if (candidates.Count > 0)
            {
                double maxArea = 0.0;
                for (int i = 0; i < candidates.Count; i++)
                    if (candidates[i].area > maxArea) maxArea = candidates[i].area;
                double areaFloor = maxArea * FaceAreaFloorRelative;

                foreach (var (face, area) in candidates)
                {
                    if (area < areaFloor) continue;
                    if (area > bestArea)
                    {
                        best = face;
                        bestArea = area;
                    }
                }
            }

            return best;
        }

        /// <summary>
        /// Phase 14.1.2: detect a plate via twin-face topology rather than
        /// planarity. Returns the larger-area face of the anti-parallel
        /// face-pair with the largest combined area, plus the average plane
        /// between the two faces' centroids and normals — the "ideal flat
        /// plate" projection plane that sits exactly between slightly-curved
        /// top and bottom faces.
        ///
        /// Twin criteria (all three required):
        ///   1. Normals anti-parallel within <see cref="TwinAntiparallelCosThreshold"/>
        ///      (default ≈ 10°). Plate top/bottom face normals point in
        ///      opposite directions.
        ///   2. Areas similar within <see cref="TwinAreaRatioMin"/> (default
        ///      0.7). Plate top/bottom are near-equal in area; firmly rejects
        ///      edge-strip / plate pairs (ratio ≈ 0.01).
        ///   3. Perpendicular separation ≤ <see cref="TwinSeparationRatioMax"/>
        ///      × max-face-bbox-diagonal (default 0.1). Plates are wide and
        ///      thin (ratio ≪ 0.1); cubes and chunky parts have ratio ≥ 0.16
        ///      and are correctly rejected.
        ///
        /// Returns null if no qualifying pair exists — caller falls back to
        /// <see cref="FindLargestPlanarFace"/> for surfaces, non-plate solids,
        /// or extreme plate geometries (heavy taper, asymmetric trim).
        /// </summary>
        private static (BrepFace Face, Plane Plane)? FindTwinPlateFace(Brep brep, double tolerance)
        {
            // Inventory: gather (face, centroid, unit-normal, area, max-dim)
            // for every face that survives DuplicateFace + AreaMassProperties.
            // Centroid is area-weighted (AreaMassProperties.Centroid); normal
            // is the surface normal at the parameter-space center, sufficient
            // for near-planar plate faces (curvature ≪ 90°). Max-dim is the
            // face bbox diagonal — orientation-agnostic single scalar.
            var inventory = new List<(BrepFace face, Point3d centroid, Vector3d normal, double area, double maxDim)>();
            foreach (var face in brep.Faces)
            {
                var faceBrep = face.DuplicateFace(false);
                if (faceBrep == null) continue;
                var amp = AreaMassProperties.Compute(faceBrep);
                if (amp == null) continue;

                double uMid = face.Domain(0).Mid;
                double vMid = face.Domain(1).Mid;
                var normal = face.NormalAt(uMid, vMid);
                if (!normal.Unitize()) continue;   // degenerate face

                var bb = faceBrep.GetBoundingBox(true);
                double maxDim = bb.Diagonal.Length;
                if (maxDim <= 0) continue;

                inventory.Add((face, amp.Centroid, normal, amp.Area, maxDim));
            }

            if (inventory.Count < 2) return null;

            // Pairing pass: O(n²) over inventory. Track the best (largest
            // combined area) pair that satisfies all three twin criteria.
            BrepFace bestFace = null;
            Plane bestPlane = Plane.Unset;
            double bestCombinedArea = -1;

            for (int i = 0; i < inventory.Count; i++)
            {
                var a = inventory[i];
                for (int j = i + 1; j < inventory.Count; j++)
                {
                    var b = inventory[j];

                    // 1. Anti-parallel: dot of unit normals ≤ −cos(10°).
                    double dot = a.normal * b.normal;
                    if (dot > -TwinAntiparallelCosThreshold) continue;

                    // 2. Area similarity: min/max ≥ 0.7.
                    double minA = Math.Min(a.area, b.area);
                    double maxA = Math.Max(a.area, b.area);
                    if (maxA <= 0 || (minA / maxA) < TwinAreaRatioMin) continue;

                    // 3. Perpendicular separation ≤ 0.1 × max-face-dim.
                    var delta = b.centroid - a.centroid;
                    double sep = Math.Abs(delta * a.normal);
                    double maxDim = Math.Max(a.maxDim, b.maxDim);
                    if (maxDim <= 0 || (sep / maxDim) > TwinSeparationRatioMax) continue;

                    double combined = a.area + b.area;
                    if (combined > bestCombinedArea)
                    {
                        // Pick the larger-area face of the pair; either is
                        // geometrically equivalent for plate extraction
                        // since the projection plane is the average.
                        bestFace = (a.area >= b.area) ? a.face : b.face;
                        bestCombinedArea = combined;

                        // Average plane: origin = midpoint of centroids,
                        // normal = (a.normal − b.normal) / 2 (since
                        // b.normal ≈ −a.normal, this averages to ≈ a.normal
                        // with small misalignments cancelled out).
                        var midOrigin = new Point3d(
                            (a.centroid.X + b.centroid.X) * 0.5,
                            (a.centroid.Y + b.centroid.Y) * 0.5,
                            (a.centroid.Z + b.centroid.Z) * 0.5);
                        var avgNormal = a.normal - b.normal;
                        if (!avgNormal.Unitize()) continue;
                        bestPlane = new Plane(midOrigin, avgNormal);
                    }
                }
            }

            if (bestFace == null) return null;
            return (bestFace, bestPlane);
        }

        private static Polygon FaceToPolygon(BrepFace face, double discretizeTol, double angleTolRad, double modelTol, double planarityTol, Plane? overridePlane, out Curve outerCurveOut, List<Curve> innerLoopsOut)
        {
            outerCurveOut = null;

            // Phase 14.1.2: prefer caller-supplied plane when available — the
            // twin-face path computes an average plane between two anti-parallel
            // plate faces and passes it in here, bypassing TryGetPlane on a
            // face that may be more curved than any planarity tolerance would
            // accept. When overridePlane is null (single-surface / fallback
            // path), fall back to Phase 14.1.1's loosened-tolerance plane fit.
            Plane facePlane;
            if (overridePlane.HasValue)
            {
                facePlane = overridePlane.Value;
            }
            else
            {
                if (!face.TryGetPlane(out facePlane, planarityTol)) return null;
            }

            var faceBrep = face.DuplicateFace(false);
            if (faceBrep == null) return null;

            Curve outerLoop = null;
            var rawInnerLoops = new List<Curve>();
            foreach (var loop in faceBrep.Loops)
            {
                if (loop.LoopType == BrepLoopType.Outer)
                {
                    if (outerLoop == null) outerLoop = loop.To3dCurve();
                }
                else if (loop.LoopType == BrepLoopType.Inner)
                {
                    var c = loop.To3dCurve();
                    if (c != null) rawInnerLoops.Add(c);
                }
            }
            if (outerLoop == null) return null;

            var outerPoly = CurveToPolygonOnPlane(outerLoop, facePlane, discretizeTol, angleTolRad);
            if (outerPoly == null) return null;

            // Phase 9a: route the outer's native curve through the same
            // plane-space projection inner loops use. Null on projection
            // failure — renderer falls back to the polygon path.
            outerCurveOut = ProjectCurveToPlaneSpace(outerLoop, facePlane, modelTol);

            foreach (var c in rawInnerLoops)
            {
                var projected = ProjectCurveToPlaneSpace(c, facePlane, modelTol);
                if (projected != null) innerLoopsOut.Add(projected);
            }

            return outerPoly;
        }

        // ------------------------------------------------------------------
        // Step 2 support: fully planar Brep
        // ------------------------------------------------------------------

        private static bool IsFullyPlanar(Brep brep, double tolerance)
        {
            foreach (var face in brep.Faces)
                if (!face.IsPlanar(tolerance)) return false;
            return true;
        }

        private static bool TryGetBrepPlane(Brep brep, double tolerance, out Plane plane)
        {
            plane = Plane.Unset;
            foreach (var face in brep.Faces)
                if (face.TryGetPlane(out plane, tolerance)) return true;
            return false;
        }

        private static Polygon FlatBrepToPolygonOnPlane(
            Brep flatBrep, Plane plane, double joinTol, double discretizeTol, double angleTolRad,
            double modelTol, out Curve outerCurveOut, List<Curve> innerLoopsOut)
        {
            outerCurveOut = null;

            var nakedEdges = new List<Curve>();
            foreach (var edge in flatBrep.Edges)
            {
                if (edge.Valence == EdgeAdjacency.Naked)
                {
                    var c = edge.DuplicateCurve();
                    if (c != null) nakedEdges.Add(c);
                }
            }
            if (nakedEdges.Count == 0) return null;

            var joined = Curve.JoinCurves(nakedEdges, joinTol);
            if (joined == null || joined.Length == 0) return null;

            Curve outer = null;
            double outerArea = -1;
            var closedLoops = new List<Curve>();
            foreach (var loop in joined)
            {
                if (!loop.IsClosed) continue;
                closedLoops.Add(loop);
                var amp = AreaMassProperties.Compute(loop);
                double area = amp?.Area ?? 0;
                if (area > outerArea) { outer = loop; outerArea = area; }
            }

            if (outer == null) return null;

            var outerPoly = CurveToPolygonOnPlane(outer, plane, discretizeTol, angleTolRad);
            if (outerPoly == null) return null;

            // Phase 9a: route the outer's native curve through plane-space
            // projection alongside the polygon. Null on projection failure —
            // renderer falls back to the polygon path.
            outerCurveOut = ProjectCurveToPlaneSpace(outer, plane, modelTol);

            foreach (var loop in closedLoops)
            {
                if (ReferenceEquals(loop, outer)) continue;
                var projected = ProjectCurveToPlaneSpace(loop, plane, modelTol);
                if (projected != null) innerLoopsOut.Add(projected);
            }

            return outerPoly;
        }

        // ------------------------------------------------------------------
        // Step 4 support: Squish for freeform / near-flat surfaces
        // ------------------------------------------------------------------

        /// <summary>
        /// Find the largest face on the Brep regardless of planarity. Used as the squish
        /// fallback face when earlier strategies fail.
        /// </summary>
        private static BrepFace FindLargestFace(Brep brep)
        {
            BrepFace best = null;
            double bestArea = -1;
            foreach (var face in brep.Faces)
            {
                var faceBrep = face.DuplicateFace(false);
                if (faceBrep == null) continue;
                var amp = AreaMassProperties.Compute(faceBrep);
                if (amp == null) continue;

                if (amp.Area > bestArea)
                {
                    best = face;
                    bestArea = amp.Area;
                }
            }
            return best;
        }

        /// <summary>
        /// Squish the trimmed face to a flat mesh, then extract the mesh's outer boundary
        /// as a 2D polygon. Meshing the trimmed face (not face.UnderlyingSurface()) is critical:
        /// the underlying surface of a Patch or trimmed NURBS extends beyond the visible face,
        /// and SquishSurface on it produces a wrong-shaped output.
        /// </summary>
        private static Polygon SquishFaceToPolygon(BrepFace face, double discretizeTol, double angleTolRad, out Curve outerCurveOut, List<Curve> innerLoopsOut)
        {
            // Phase 9a: Squish has no native source curve — output is meshed-and-
            // squished polylines. Renderer falls back to drawing the placed
            // polygon directly for parts that came through this path.
            outerCurveOut = null;

            var faceBrep = face.DuplicateFace(false);
            if (faceBrep == null) return null;

            // Mesh the trimmed face.
            var meshingParams = MeshingParameters.QualityRenderMesh;
            var meshes = Mesh.CreateFromBrep(faceBrep, meshingParams);
            if (meshes == null || meshes.Length == 0) return null;

            var mesh3d = new Mesh();
            foreach (var m in meshes) mesh3d.Append(m);
            if (mesh3d.Vertices.Count == 0) return null;

            var parameters = new SquishParameters();

            Mesh squishedMesh;
            try
            {
                using (var squisher = new Squisher())
                {
                    squishedMesh = squisher.SquishMesh(parameters, mesh3d);
                }
            }
            catch
            {
                return null;
            }

            if (squishedMesh == null || squishedMesh.Vertices.Count == 0) return null;

            // Extract outer boundary from the squished 2D mesh's naked edges.
            var nakedEdges = squishedMesh.GetNakedEdges();
            if (nakedEdges == null || nakedEdges.Length == 0) return null;

            // Longest naked-edge polyline is the outer boundary; shorter ones would be holes.
            Polyline outer = nakedEdges[0];
            double outerLength = outer.Length;
            for (int i = 1; i < nakedEdges.Length; i++)
            {
                double len = nakedEdges[i].Length;
                if (len > outerLength) { outer = nakedEdges[i]; outerLength = len; }
            }

            if (outer.Count < 3) return null;

            var points = new List<Point2D>(outer.Count);
            int count = outer.Count;
            if (outer.IsClosed && count > 3) count--;
            for (int i = 0; i < count; i++)
            {
                points.Add(new Point2D(outer[i].X, outer[i].Y));
            }
            if (points.Count < 3) return null;

            Polygon outerPoly;
            try { outerPoly = new Polygon(points); }
            catch (ArgumentException) { return null; }

            // Squished mesh is already 2D — emit the non-longest naked edges as
            // closed PolylineCurves at z=0. Squish-path dimensional caveat is
            // already surfaced via SquishWarning; same caveat covers these holes.
            for (int i = 0; i < nakedEdges.Length; i++)
            {
                var edge = nakedEdges[i];
                if (ReferenceEquals(edge, outer)) continue;
                if (edge.Count < 3) continue;

                var pl = new Polyline(edge.Count + 1);
                for (int j = 0; j < edge.Count; j++)
                    pl.Add(edge[j].X, edge[j].Y, 0);
                if (!pl.IsClosed) pl.Add(pl[0]);
                if (pl.Count < 4) continue;

                innerLoopsOut.Add(new PolylineCurve(pl));
            }

            return outerPoly;
        }

        // ------------------------------------------------------------------
        // Shared: curve -> polygon via projection onto a given plane
        // ------------------------------------------------------------------

        /// <summary>
        /// Project a curve lying on <paramref name="plane"/> into the plane's UV frame,
        /// expressed as a Rhino curve in world XY at Z=0. Used by Phase 7b inner-loop
        /// preservation, Phase 7b.2 upgrade: <see cref="Transform.PlaneToPlane"/> is a
        /// rigid transform — the curve's native subclass (ArcCurve, LineCurve,
        /// PolylineCurve, NurbsCurve, …) survives, so circles stay circles, arcs stay
        /// arcs, all the way through <see cref="PolygonToCurve.ToCurveFromOriginal"/>
        /// and into the DXF export. No per-vertex sampling, no over-tessellation —
        /// a 3" circle in the source emits one ArcCurve, not a 1024-vertex polyline.
        ///
        /// Coordinate equivalence with the outer's <see cref="CurveToPolygonOnPlane"/>
        /// helper is exact: <c>PlaneToPlane(plane, WorldXY)</c> maps
        /// <c>plane.PointAt(u, v) → (u, v, 0)</c> in world XYZ, the same (u, v) values
        /// that <see cref="Plane.ClosestParameter"/> returns per-vertex in the
        /// polygon-projection path.
        ///
        /// Returns null if duplication or transformation fails — caller skips the loop,
        /// matching the same fall-through pattern <see cref="FlattenRaw"/> uses for
        /// other helper failures.
        /// </summary>
        private static Curve ProjectCurveToPlaneSpace(Curve curve, Plane plane, double modelTol)
        {
            if (curve == null) return null;

            // Phase 15.1: orthographically project the curve onto `plane`
            // BEFORE any further processing. The Transform.PlaneToPlane call
            // below is a rigid 3D coordinate-frame transform — it preserves
            // the out-of-plane (perpendicular) component of every point.
            // Curves that lie exactly on `plane` are fine, but curves that
            // bulge off `plane` (curved-face edges, and especially Phase
            // 14.1.2's twin-plate average plane where edge curves deviate
            // up to half the plate thickness) retain their 3D shape after
            // the rigid transform — landing in the world-XY result with
            // non-zero Z extent. ProjectToPlane is a perpendicular
            // projection (orthographic shadow on the plane), which is
            // exactly what plate flattening means and what the polygon
            // path produces via Plane.ClosestParameter. Calling it here
            // aligns the native curve with the polygon's geometric content.
            // Graceful fallback to the un-projected curve on null —
            // pre-15.1 output (small Z drift) is better than dropping the
            // curve entirely.
            var flattened = Curve.ProjectToPlane(curve, plane) ?? curve;

            var planeToXY = Transform.PlaneToPlane(plane, Plane.WorldXY);

            // Phase 7c.5: for PolyCurve inputs, attempt to coalesce constituent
            // segments via Explode→JoinCurves before probing. BrepLoop.To3dCurve()
            // commonly returns a polycurve of two NURBS half-circles for circular
            // trims; TryGetCircle doesn't aggregate polycurve segments before
            // testing, so each half individually fails the circle check. Joining
            // the exploded segments produces a single curve whose whole-curve
            // geometry TryGetCircle can identify as a circle within modelTol,
            // promoting the output to ArcCurve and a DXF-native CIRCLE entity.
            // Non-PolyCurve inputs skip this block and proceed to the direct
            // probes below. Disposal of segments / joined results follows the
            // file's existing convention (e.g. FlatBrepToPolygonOnPlane's
            // JoinCurves call): rely on GC, no explicit cleanup.
            if (flattened is PolyCurve polyCurve)
            {
                var segments = polyCurve.Explode();
                if (segments != null && segments.Length > 0)
                {
                    var joined = Curve.JoinCurves(segments, modelTol);
                    if (joined != null && joined.Length == 1)
                    {
                        var coalesced = joined[0];

                        // Phase 7c.5.2: if the joined result is still a PolyCurve,
                        // convert to a single multi-span NurbsCurve. JoinCurves on
                        // two NURBS half-circles returns the result as a closed
                        // PolyCurve, and TryGetCircle on a PolyCurve suffers the
                        // same segment-aggregation limitation that drove 7c.5 in
                        // the first place. ToNurbsCurve consolidates the
                        // polycurve's segments into one parametric curve whose
                        // evaluated geometry TryGetCircle/TryGetArc can probe
                        // directly. Standard Rhino idiom. Skip when joined[0] is
                        // already a NurbsCurve / ArcCurve — no conversion needed.
                        if (coalesced is PolyCurve)
                        {
                            var nurbsForm = coalesced.ToNurbsCurve();
                            if (nurbsForm != null) coalesced = nurbsForm;
                        }

                        if (coalesced.TryGetCircle(out Circle pcCircle, modelTol))
                        {
                            var arcCurve = new ArcCurve(pcCircle);
                            if (!arcCurve.Transform(planeToXY)) return null;
                            return arcCurve;
                        }
                        if (coalesced.TryGetArc(out Arc pcArc, modelTol))
                        {
                            var arcCurve = new ArcCurve(pcArc);
                            if (!arcCurve.Transform(planeToXY)) return null;
                            return arcCurve;
                        }
                    }
                }
            }

            // Probe for native shapes BEFORE the rigid transform: BrepLoop.To3dCurve()
            // hands back NURBS spans even when the trim is geometrically a perfect
            // circle or arc, so a direct DuplicateCurve preserves the NURBS form
            // (22 control points for a circle). Recovering Circle/Arc here promotes
            // those NURBS to ArcCurve, which survives the transform and exports as
            // a native DXF circle/arc entity. Circle probed first because a full
            // circle also matches TryGetArc — circle is the more specific verdict.
            if (flattened.TryGetCircle(out Circle circle, modelTol))
            {
                var arcCurve = new ArcCurve(circle);
                if (!arcCurve.Transform(planeToXY)) return null;
                return arcCurve;
            }
            if (flattened.TryGetArc(out Arc arc, modelTol))
            {
                var arcCurve = new ArcCurve(arc);
                if (!arcCurve.Transform(planeToXY)) return null;
                return arcCurve;
            }

            // General case — preserves native subclass through Transform.
            var working = flattened.DuplicateCurve();
            if (working == null) return null;
            if (!working.Transform(planeToXY)) return null;
            return working;
        }

        private static Polygon CurveToPolygonOnPlane(Curve curve, Plane plane, double discretizeTol, double angleTolRad)
        {
            Polyline polyline;
            if (!curve.TryGetPolyline(out polyline))
            {
                var polylineCurve = curve.ToPolyline(
                    mainSegmentCount: 0,
                    subSegmentCount: 0,
                    maxAngleRadians: angleTolRad,
                    maxChordLengthRatio: 0,
                    maxAspectRatio: 0,
                    tolerance: discretizeTol,
                    minEdgeLength: 0,
                    maxEdgeLength: 0,
                    keepStartPoint: true);
                if (polylineCurve == null || !polylineCurve.TryGetPolyline(out polyline))
                    return null;
            }

            if (polyline == null || polyline.Count < 3) return null;

            var points = new List<Point2D>(polyline.Count);
            int count = polyline.Count;
            if (polyline.IsClosed && count > 3) count--;
            for (int i = 0; i < count; i++)
            {
                var pt3 = polyline[i];
                double u, v;
                if (!plane.ClosestParameter(pt3, out u, out v)) return null;
                points.Add(new Point2D(u, v));
            }
            if (points.Count < 3) return null;

            try { return new Polygon(points); }
            catch (ArgumentException) { return null; }
        }

        // ------------------------------------------------------------------
        // Unroller for developable curved parts
        // ------------------------------------------------------------------

        private static Brep Unroll(Brep brep, double absoluteTolerance, double angleToleranceRadians)
        {
            var unroller = new Unroller(brep)
            {
                AbsoluteTolerance = absoluteTolerance,
                RelativeTolerance = absoluteTolerance,
                ExplodeOutput = false
            };

            Curve[] unrolledCurves;
            Point3d[] unrolledPoints;
            TextDot[] unrolledDots;
            var unrolled = unroller.PerformUnroll(out unrolledCurves, out unrolledPoints, out unrolledDots);

            if (unrolled == null || unrolled.Length == 0) return null;
            if (unrolled.Length == 1) return unrolled[0];

            Brep largest = unrolled[0];
            double largestArea = AreaMassProperties.Compute(largest)?.Area ?? 0;
            for (int i = 1; i < unrolled.Length; i++)
            {
                double area = AreaMassProperties.Compute(unrolled[i])?.Area ?? 0;
                if (area > largestArea) { largest = unrolled[i]; largestArea = area; }
            }
            return largest;
        }
    }
}