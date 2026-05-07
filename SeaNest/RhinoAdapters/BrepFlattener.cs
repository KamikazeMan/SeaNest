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
        /// Callback invoked when a part was flattened using Squish. Used by the command layer
        /// to inform the user that dimensions may be approximate for curved parts.
        /// </summary>
        public static Action<string> SquishWarning { get; set; }

        public static Polygon Flatten(Brep brep, RhinoDoc doc)
        {
            return FlattenRaw(brep, doc);
        }

        private static Polygon FlattenRaw(Brep brep, RhinoDoc doc)
        {
            if (brep == null) throw new ArgumentNullException(nameof(brep));
            if (doc == null) throw new ArgumentNullException(nameof(doc));

            double modelTol = doc.ModelAbsoluteTolerance;
            double discretizeTol = modelTol * DiscretizationToleranceFactor;
            double angleTolRad = RhinoMath.ToRadians(UnrollAngleToleranceDegrees);

            // Step 1: Thickened plate — largest coplanar face region.
            var plateFace = FindLargestPlanarFace(brep, modelTol);
            if (plateFace != null)
            {
                var poly = FaceToPolygon(plateFace, discretizeTol, angleTolRad);
                if (poly != null) return poly;
            }

            // Step 2: Fully flat open Brep.
            if (IsFullyPlanar(brep, modelTol))
            {
                Plane plane;
                if (TryGetBrepPlane(brep, modelTol, out plane))
                {
                    var poly = FlatBrepToPolygonOnPlane(brep, plane, modelTol, discretizeTol, angleTolRad);
                    if (poly != null) return poly;
                }
            }

            // Step 3: Developable curved Brep — unroll.
            double unrollTol = modelTol * UnrollToleranceFactor;
            var unrolled = Unroll(brep, unrollTol, angleTolRad);
            if (unrolled != null)
            {
                var poly = FlatBrepToPolygonOnPlane(unrolled, Plane.WorldXY, modelTol, discretizeTol, angleTolRad);
                if (poly != null) return poly;
            }

            // Step 4: Fallback — squish the largest face regardless of planarity.
            var squishFace = FindLargestFace(brep);
            if (squishFace != null)
            {
                var poly = SquishFaceToPolygon(squishFace, discretizeTol, angleTolRad);
                if (poly != null)
                {
                    SquishWarning?.Invoke(
                        "Squish flattening used — dimensions may be approximate for curved parts.");
                    return poly;
                }
            }

            return null;
        }

        // ------------------------------------------------------------------
        // Step 1 support: largest coplanar-face region
        // ------------------------------------------------------------------

        private static BrepFace FindLargestPlanarFace(Brep brep, double tolerance)
        {
            var entries = new List<(BrepFace face, Plane plane, double area)>();
            foreach (var face in brep.Faces)
            {
                if (!face.IsPlanar(tolerance)) continue;
                Plane plane;
                if (!face.TryGetPlane(out plane)) continue;

                var faceBrep = face.DuplicateFace(false);
                if (faceBrep == null) continue;
                var amp = AreaMassProperties.Compute(faceBrep);
                if (amp == null) continue;

                entries.Add((face, plane, amp.Area));
            }
            if (entries.Count == 0) return null;

            double normalTol = 1e-4;
            double offsetTol = Math.Max(tolerance, 1e-6) * 10.0;

            var groups = new List<(Vector3d normal, double offset, List<(BrepFace face, double area)> faces, double totalArea)>();

            foreach (var e in entries)
            {
                Vector3d n = e.plane.Normal;
                n.Unitize();
                double offset = n * (Vector3d)e.plane.Origin;

                bool matched = false;
                for (int i = 0; i < groups.Count; i++)
                {
                    var g = groups[i];
                    double dot = g.normal * n;
                    bool parallel = Math.Abs(dot) > 1.0 - normalTol;
                    if (!parallel) continue;

                    double gOffset = dot > 0 ? g.offset : -g.offset;
                    if (Math.Abs(gOffset - offset) > offsetTol) continue;

                    g.faces.Add((e.face, e.area));
                    g.totalArea += e.area;
                    groups[i] = g;
                    matched = true;
                    break;
                }
                if (!matched)
                {
                    groups.Add((n, offset, new List<(BrepFace, double)> { (e.face, e.area) }, e.area));
                }
            }

            int bestIdx = 0;
            for (int i = 1; i < groups.Count; i++)
            {
                if (groups[i].totalArea > groups[bestIdx].totalArea + tolerance)
                    bestIdx = i;
                else if (Math.Abs(groups[i].totalArea - groups[bestIdx].totalArea) < tolerance &&
                         groups[i].normal.Z > groups[bestIdx].normal.Z)
                    bestIdx = i;
            }

            var winning = groups[bestIdx];
            BrepFace winnerFace = winning.faces[0].face;
            double winnerArea = winning.faces[0].area;
            for (int i = 1; i < winning.faces.Count; i++)
            {
                if (winning.faces[i].area > winnerArea)
                {
                    winnerFace = winning.faces[i].face;
                    winnerArea = winning.faces[i].area;
                }
            }
            return winnerFace;
        }

        private static Polygon FaceToPolygon(BrepFace face, double discretizeTol, double angleTolRad)
        {
            Plane facePlane;
            if (!face.TryGetPlane(out facePlane)) return null;

            var faceBrep = face.DuplicateFace(false);
            if (faceBrep == null) return null;

            Curve outerLoop = null;
            foreach (var loop in faceBrep.Loops)
            {
                if (loop.LoopType == BrepLoopType.Outer)
                {
                    outerLoop = loop.To3dCurve();
                    break;
                }
            }
            if (outerLoop == null) return null;

            return CurveToPolygonOnPlane(outerLoop, facePlane, discretizeTol, angleTolRad);
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
            Brep flatBrep, Plane plane, double joinTol, double discretizeTol, double angleTolRad)
        {
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
            foreach (var loop in joined)
            {
                if (!loop.IsClosed) continue;
                var amp = AreaMassProperties.Compute(loop);
                double area = amp?.Area ?? 0;
                if (area > outerArea) { outer = loop; outerArea = area; }
            }
            if (outer == null) return null;

            return CurveToPolygonOnPlane(outer, plane, discretizeTol, angleTolRad);
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
        private static Polygon SquishFaceToPolygon(BrepFace face, double discretizeTol, double angleTolRad)
        {
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

            try { return new Polygon(points); }
            catch (ArgumentException) { return null; }
        }

        // ------------------------------------------------------------------
        // Shared: curve -> polygon via projection onto a given plane
        // ------------------------------------------------------------------

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