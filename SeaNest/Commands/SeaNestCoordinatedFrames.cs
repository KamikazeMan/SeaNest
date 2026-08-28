using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Nesting;
using SeaNest.Nesting.Core.Overlap;
using SeaNest.RhinoAdapters;

namespace SeaNest.Commands
{
    // Phase 30 (increment 1): coordinated frame nester — plumbing.
    //
    // Selects the large hull frames (Irregular parts with bbox area >= 5% of
    // sheet area, the same set NestingEngine calls CriticalPartIndices), places
    // them greedily at FIXED orientation (no rotation this increment) at the
    // best NFP-boundary position against the already-placed frames using a
    // placeholder bottom-left fit score, then hands the placed frames to the
    // existing engine via PlaceAllWithPreplaced to fill stringers/small parts
    // (and spill any frame that wouldn't fit). Validates no-overlap, draws, and
    // writes a deterministic Desktop\phase30_frames.txt.
    //
    // Pure NFP + engine (no OR-Tools) -> builds on net7.0 and net48; no guard.
    // Rotation search is increment 2; the real profile-contact tightness metric
    // is increment 3.
    public class SeaNestCoordinatedFrames : Command
    {
        public override string EnglishName => "SeaNestCoordinatedFrames";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            // No timestamp in the file: the determinism check requires repeated
            // runs to produce byte-identical phase30_frames.txt.
            var diag = new List<string> { "Phase 30 coordinated-frames (increment 1)", "" };

            try
            {
                // --- Input path: re-nest already-placed parts, matching
                // SeaNestReNestCommand EXACTLY. The user selects the grouped
                // nested layout (group-select pulls in all members); closed
                // curves are partitioned into outer parts vs holes and each
                // outer is converted to a Polygon via the same CurveToPolygon
                // path ReNest uses. (Open curves / labels never become parts in
                // ReNest either, so they are not part of the polygon set.)
                bool isMetric = doc.ModelUnitSystem == UnitSystem.Millimeters
                             || doc.ModelUnitSystem == UnitSystem.Centimeters
                             || doc.ModelUnitSystem == UnitSystem.Meters;
                double inToModel = isMetric
                    ? RhinoMath.UnitScale(UnitSystem.Inches, doc.ModelUnitSystem)
                    : 1.0;

                double sheetW, sheetH, margin, spacing;
                if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet width", 96.0 * inToModel, out sheetW)) return Result.Cancel;
                if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet height", 48.0 * inToModel, out sheetH)) return Result.Cancel;
                if (!SeaNestNestCommand.PromptForDouble(doc, "Margin", 0.25 * inToModel, out margin)) return Result.Cancel;
                if (!SeaNestNestCommand.PromptForDouble(doc, "Spacing", 0.25 * inToModel, out spacing)) return Result.Cancel;
                diag.Add($"Sheet {sheetW:F2} x {sheetH:F2}, spacing {spacing:F3}, margin {margin:F2}");

                var go = new GetObject();
                go.SetCommandPrompt("Select the grouped nested parts to re-nest (frames + everything else)");
                go.GeometryFilter = ObjectType.Curve | ObjectType.Annotation;
                go.GroupSelect = true;           // selecting the group selects all members
                go.SubObjectSelect = false;
                go.GetMultiple(1, 0);
                if (go.CommandResult() != Result.Success)
                    return go.CommandResult();

                double modelTol = doc.ModelAbsoluteTolerance;
                double discretizeTol = modelTol * 0.1;
                double angleTolRad = RhinoMath.ToRadians(0.5);

                // Closed curves are the part outlines (open curves = etch/scribe,
                // annotations = labels; neither becomes a part — same as ReNest).
                var closedCurves = new List<Curve>();
                for (int i = 0; i < go.ObjectCount; i++)
                {
                    if (go.Object(i).Geometry() is Curve cv && cv.IsClosed)
                        closedCurves.Add(cv);
                }
                if (closedCurves.Count == 0)
                {
                    diag.Add("No closed curves selected — nothing to nest.");
                    Flush(diag);
                    return Result.Cancel;
                }

                // Topology partition. containedBy[i] = curves that contain i.
                // Immediate parent = smallest containing curve by bbox area.
                // Outers = curves with no parent. Non-outers walk their parent
                // chain up to a root outer and become attached HOLES of that
                // root, drawn along with the outer at placement time (was:
                // silently discarded).
                //
                // Containment robustness: the ReNest-verbatim version used
                // Curve.PlanarClosedCurveRelationship on WorldXY only. That is
                // strict about tangent plane and misses interior curves whose
                // plane drifts slightly from WorldXY (Z offset from a prior
                // operation, tilted parent frame). Symptom on the catamaran
                // job: 3 pipe-hole circles inside a frame were reported
                // Disjoint from every other curve and became phantom
                // standalone parts. Fix: fast XY bbox filter → PCR primary →
                // Curve.Contains centroid fallback when PCR says Disjoint
                // despite bbox indicating containment.
                var topologyPlane = Plane.WorldXY;
                double topologyTol = modelTol * 10.0;
                int nC = closedCurves.Count;
                var containedBy = new List<int>[nC];
                var bboxes = new BoundingBox[nC];
                var bboxArea = new double[nC];
                for (int i = 0; i < nC; i++)
                {
                    containedBy[i] = new List<int>();
                    bboxes[i] = closedCurves[i].GetBoundingBox(true);
                    var d = bboxes[i].Diagonal;
                    bboxArea[i] = d.X * d.Y;
                }
                for (int i = 0; i < nC - 1; i++)
                    for (int j = i + 1; j < nC; j++)
                    {
                        int cmp = TestContainment(
                            closedCurves[i], closedCurves[j],
                            bboxes[i], bboxes[j],
                            topologyPlane, topologyTol);
                        if (cmp < 0) containedBy[i].Add(j);
                        else if (cmp > 0) containedBy[j].Add(i);
                    }
                var parent = new int[nC];
                for (int i = 0; i < nC; i++)
                {
                    if (containedBy[i].Count == 0) { parent[i] = -1; continue; }
                    int smallest = containedBy[i][0];
                    for (int k = 1; k < containedBy[i].Count; k++)
                        if (bboxArea[containedBy[i][k]] < bboxArea[smallest])
                            smallest = containedBy[i][k];
                    parent[i] = smallest;
                }
                var outers = new List<int>();
                for (int i = 0; i < nC; i++)
                    if (parent[i] == -1) outers.Add(i);

                // Root-outer for each non-outer curve: walk parent[] to the
                // topmost ancestor with parent == -1. Handles nested-hole /
                // island-inside-hole cases naturally by attaching every
                // interior curve to the outermost frame it lives in — that
                // frame owns the physical cutting.
                var rootOuterOf = new int[nC];
                for (int i = 0; i < nC; i++)
                {
                    int cur = i;
                    while (parent[cur] != -1) cur = parent[cur];
                    rootOuterOf[i] = cur;
                }

                // Group hole curve indices by their root outer.
                var holesByOuter = new Dictionary<int, List<int>>();
                foreach (int outerIdx in outers) holesByOuter[outerIdx] = new List<int>();
                for (int i = 0; i < nC; i++)
                {
                    if (parent[i] == -1) continue;
                    holesByOuter[rootOuterOf[i]].Add(i);
                }

                // Convert each outer to a Polygon (verbatim CurveToPolygon
                // path). Alongside each polygon, capture the plane-mapped
                // hole curves in an index-parallel list so the draw step can
                // apply the placement's transform to each hole via
                // PolygonToCurve.ToCurveFromOriginal.
                var polygons = new List<Polygon>();
                var holeCurvesPerPart = new List<List<Curve>>();
                var partSourceCurveIdx = new List<int>(); // diagnostic only
                foreach (int outerIdx in outers)
                {
                    var outerCurve = closedCurves[outerIdx];
                    if (!outerCurve.TryGetPlane(out Plane op, discretizeTol * 10.0))
                        op = Plane.WorldXY;
                    var converted = CurveToPolygon(outerCurve, op, discretizeTol, angleTolRad, MaxVertices);
                    if (converted == null) continue;

                    polygons.Add(converted.Value.polygon);
                    partSourceCurveIdx.Add(outerIdx);

                    // Map every attached hole into the outer's plane frame so
                    // it shares WorldXY with the polygon and lines up under
                    // the same placement transform.
                    var mappedHoles = new List<Curve>();
                    foreach (int holeIdx in holesByOuter[outerIdx])
                    {
                        var mapped = MapCurveToLocalFrame(closedCurves[holeIdx], op);
                        if (mapped != null) mappedHoles.Add(mapped);
                    }
                    holeCurvesPerPart.Add(mappedHoles);
                }
                if (polygons.Count == 0)
                {
                    diag.Add("No curves could be converted. Aborting.");
                    Flush(diag);
                    return Result.Failure;
                }
                int totalHolesAttached = holeCurvesPerPart.Sum(l => l.Count);
                diag.Add($"Read {polygons.Count} outer part(s) from the nested layout " +
                         $"({nC} closed curve(s); {totalHolesAttached} hole(s) attached to parents).");
                for (int k = 0; k < polygons.Count; k++)
                {
                    if (holeCurvesPerPart[k].Count > 0)
                        diag.Add($"  part {k} (curve #{partSourceCurveIdx[k]}): {holeCurvesPerPart[k].Count} hole(s) attached");
                }

                var request = new NestRequest(
                    polygons, sheetW, sheetH, 0.0, margin, spacing,
                    RotationStep.Every90, NestingAlgorithm.NFP_Annealed,
                    allowMirror: false, TimeSpan.FromSeconds(60));

                OrientedPart.BuildAll(
                    polygons, request.RotationStepDegrees, request.AllowMirror,
                    out _, out var orientationsByPart);
                var cache = new NfpCache(request.Spacing);
                var engine = new NfpPlacementEngine(request, orientationsByPart, cache);

                // Route the frame nester's per-frame progress to the Rhino
                // command line and keep Rhino responsive (the increment-1 run
                // completed but looked frozen because it ran silently).
                engine.DiagnosticLog = msg =>
                {
                    RhinoApp.WriteLine(msg);
                    RhinoApp.Wait();
                };

                // Frame set: Irregular parts with bbox area >= 5% of sheet area
                // (matches NestingEngine.cs:280-291 CriticalPartIndices).
                double sheetArea = sheetW * sheetH;
                double criticalThreshold = sheetArea * 0.05;
                var frames = new List<int>();
                for (int i = 0; i < polygons.Count; i++)
                {
                    if (PartClassifier.Classify(polygons[i]) == PartClass.Irregular)
                    {
                        var bb = polygons[i].BoundingBox;
                        if (bb.Width * bb.Height >= criticalThreshold)
                            frames.Add(i);
                    }
                }
                diag.Add($"Identified {frames.Count} frame(s) (Irregular, bbox >= 5% sheet): " +
                         string.Join(",", frames));

                // Everything else, largest-area first (index tiebreak) for fill.
                var frameSet = new HashSet<int>(frames);
                var remaining = Enumerable.Range(0, polygons.Count)
                    .Where(i => !frameSet.Contains(i))
                    .OrderByDescending(i => polygons[i].AbsoluteArea)
                    .ThenBy(i => i)
                    .ToList();

                // NOTE (runtime/follow-up): NFP candidate generation runs on the
                // full-resolution frame polygons (the CurveToPolygon read caps
                // each at 500 vertices). NFP on high-vertex concave frames is the
                // slow step. A faster path — simplified polygons for NFP/candidate
                // generation but full-res for the hard overlap gate and final
                // output — needs dual geometry in PlaceCoordinatedFrames and is
                // deferred so the output/gate stay full-resolution. Progress is
                // reported per frame to keep Rhino responsive meanwhile.
                var result = engine.PlaceCoordinatedFrames(frames, remaining, diag);

                // Focused validation: no overlap among placed frames.
                var framePlacements = result.Placements
                    .Where(p => frameSet.Contains(p.OriginalIndex))
                    .ToList();
                // Overlap is only meaningful WITHIN a sheet: PlacedPolygon
                // coordinates are sheet-local, so two parts on different sheets
                // both sit near the origin and would falsely register as
                // overlapping. Compare only same-sheet pairs.
                int frameOverlaps = 0;
                for (int a = 0; a < framePlacements.Count; a++)
                    for (int b = a + 1; b < framePlacements.Count; b++)
                        if (framePlacements[a].Sheet == framePlacements[b].Sheet &&
                            OverlapChecker.Overlaps(
                                framePlacements[a].PlacedPolygon,
                                framePlacements[b].PlacedPolygon))
                            frameOverlaps++;
                diag.Add($"Placed frames: {framePlacements.Count}/{frames.Count}. " +
                         $"Frame-pair overlaps: {frameOverlaps} (expect 0).");

                double placedArea = 0;
                foreach (var p in result.Placements)
                    placedArea += polygons[p.OriginalIndex].AbsoluteArea;
                double util = result.SheetCount > 0
                    ? placedArea / (result.SheetCount * sheetArea) : 0.0;

                diag.Add($"Sheets: {result.SheetCount}. Placed {result.Placements.Count}/{polygons.Count}. " +
                         $"Unplaced {result.Unplaced.Count}. Utilization {util:P1} " +
                         "(placed source area / (sheets x sheet area)).");
                if (result.Unplaced.Count > 0)
                    diag.Add($"Unplaced indices: {string.Join(",", result.Unplaced)}");

                // Draw: sheets stacked vertically; frames vs fill on separate layers.
                int frameLayer = GetOrCreateLayer(doc, "Frames_Coordinated", System.Drawing.Color.OrangeRed);
                int fillLayer = GetOrCreateLayer(doc, "Frames_Fill", System.Drawing.Color.SteelBlue);
                int sheetLayer = GetOrCreateLayer(doc, "Frames_Sheets", System.Drawing.Color.Gray);
                double stride = sheetH + 5.0;

                for (int s = 0; s < result.SheetCount; s++)
                    doc.Objects.AddCurve(
                        PolygonToCurve.SheetRectangle(sheetW, sheetH, s * stride),
                        new ObjectAttributes { LayerIndex = sheetLayer });

                foreach (var p in result.Placements)
                {
                    double yOff = p.Sheet * stride;
                    int lyr = frameSet.Contains(p.OriginalIndex) ? frameLayer : fillLayer;

                    // Draw the outer silhouette.
                    doc.Objects.AddCurve(
                        PolygonToCurve.ToCurve(p.PlacedPolygon, yOff),
                        new ObjectAttributes { LayerIndex = lyr });

                    // Draw every hole attached to this part at the same
                    // placed-and-oriented position, on the same layer as the
                    // outer so CAM cuts them together. ToCurveFromOriginal
                    // handles mirror + Transform2D + per-sheet yOffset — the
                    // exact same transformation the outer went through — so
                    // holes land where they belong INSIDE the placed frame.
                    if (p.OriginalIndex >= 0 && p.OriginalIndex < holeCurvesPerPart.Count)
                    {
                        var holes = holeCurvesPerPart[p.OriginalIndex];
                        for (int h = 0; h < holes.Count; h++)
                        {
                            var placedHole = PolygonToCurve.ToCurveFromOriginal(
                                holes[h], p, yOff);
                            if (placedHole != null)
                                doc.Objects.AddCurve(
                                    placedHole,
                                    new ObjectAttributes { LayerIndex = lyr });
                        }
                    }
                }
                doc.Views.Redraw();

                RhinoApp.WriteLine(
                    $"Coordinated frames: {frames.Count} frame(s), {result.SheetCount} sheet(s), " +
                    $"utilization {util:P1}, {frameOverlaps} frame overlaps. See phase30_frames.txt.");

                Flush(diag);
                return Result.Success;
            }
            catch (Exception ex)
            {
                diag.Add($"EXCEPTION ({ex.GetType().Name}): {ex.Message}");
                if (ex.InnerException != null)
                    diag.Add($"  Inner ({ex.InnerException.GetType().Name}): {ex.InnerException.Message}");
                diag.Add(ex.StackTrace ?? "(no stack trace)");
                Flush(diag);
                RhinoApp.WriteLine("SeaNestCoordinatedFrames failed — see phase30_frames.txt.");
                return Result.Failure;
            }
        }

        // Curve->Polygon conversion constants and helpers, copied verbatim from
        // SeaNestReNestCommand so this command's part-acquisition is identical
        // (and the production ReNest command is left untouched).
        private const int MaxVertices = 500;
        private const double EscalationFactor = 2.0;
        private const int MaxEscalations = 3;

        private static (Polygon polygon, Curve originalCurve, double finalTolerance)? CurveToPolygon(
            Curve curve, Plane plane, double discretizeTol, double angleTolRad, int maxVertices)
        {
            if (!curve.IsClosed)
                return null;

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
                if (!plane.ClosestParameter(pt3, out double u, out double v))
                    return null;
                points.Add(new Point2D(u, v));
            }
            if (points.Count < 3) return null;

            Polygon poly;
            try { poly = new Polygon(points); }
            catch (ArgumentException) { return null; }

            Polygon simplified;
            double finalTol;
            try
            {
                simplified = poly.SimplifyToTarget(
                    initialTolerance: discretizeTol,
                    maxVertices: maxVertices,
                    escalationFactor: EscalationFactor,
                    maxEscalations: MaxEscalations,
                    out finalTol);
            }
            catch (ArgumentException)
            {
                simplified = poly;
                finalTol = discretizeTol;
            }

            var localCurve = MapCurveToLocalFrame(curve, plane);
            return (simplified, localCurve, finalTol);
        }

        // Robust curve-vs-curve containment for the topology-partition step.
        //
        // Returns:
        //   -1  if `a` is inside `b`
        //   +1  if `b` is inside `a`
        //    0  if neither is inside the other (disjoint, mutually
        //        intersecting, or bbox-disjoint fast reject)
        //
        // Layering:
        //   1. XY-only bbox test as a fast necessary condition. Uses XY
        //      coordinates rather than Rhino's 3D BoundingBox.Contains
        //      because a hole with tiny Z drift from its parent's plane
        //      would fail the 3D test even though its XY footprint is
        //      fully inside — exactly the catamaran-job failure mode.
        //   2. Curve.PlanarClosedCurveRelationship — the primary test.
        //      Cheap on curves that agree with `pairPlane` and gives the
        //      final answer for the vast majority of pairs.
        //   3. Centroid Curve.Contains — fallback when PCR reports
        //      Disjoint despite bbox-inside. This catches slightly
        //      off-plane interior curves that PCR rejects. The outer's
        //      own tangent plane is used for the containment test
        //      (falling back to `pairPlane`) so a tilted parent frame's
        //      real plane drives the point-in-region query.
        //
        // Kept intentionally single-scope: no caching, no allocations for
        // the fast-reject path. Called O(n²) in the partition loop; the
        // primary cost lives in PCR/Contains inside Rhino, not this
        // dispatcher.
        private static int TestContainment(
            Curve a, Curve b,
            BoundingBox bbA, BoundingBox bbB,
            Plane pairPlane, double tol)
        {
            bool aInsideBBoxOfB = BBoxContainsXY(bbB, bbA);
            bool bInsideBBoxOfA = BBoxContainsXY(bbA, bbB);
            if (!aInsideBBoxOfB && !bInsideBBoxOfA) return 0;

            var rel = Curve.PlanarClosedCurveRelationship(a, b, pairPlane, tol);
            if (rel == RegionContainment.AInsideB) return -1;
            if (rel == RegionContainment.BInsideA) return +1;
            if (rel == RegionContainment.MutualIntersection) return 0;

            // rel == Disjoint but bbox says one is inside the other.
            // Fall back to a centroid-based point-in-region probe.
            if (aInsideBBoxOfB && CentroidInside(a, b, pairPlane, tol)) return -1;
            if (bInsideBBoxOfA && CentroidInside(b, a, pairPlane, tol)) return +1;
            return 0;
        }

        private static bool BBoxContainsXY(BoundingBox outer, BoundingBox inner)
        {
            return outer.Min.X <= inner.Min.X + 1e-12
                && inner.Max.X <= outer.Max.X + 1e-12
                && outer.Min.Y <= inner.Min.Y + 1e-12
                && inner.Max.Y <= outer.Max.Y + 1e-12;
        }

        private static bool CentroidInside(Curve inner, Curve outer, Plane fallbackPlane, double tol)
        {
            var amp = AreaMassProperties.Compute(inner);
            Point3d probe = amp != null ? amp.Centroid : inner.GetBoundingBox(true).Center;
            if (!outer.TryGetPlane(out Plane probePlane, tol * 10.0))
                probePlane = fallbackPlane;
            var cont = outer.Contains(probe, probePlane, tol);
            return cont == PointContainment.Inside || cont == PointContainment.Coincident;
        }

        private static Curve MapCurveToLocalFrame(Curve curve, Plane plane)
        {
            var local = curve.DuplicateCurve();
            var projected = Curve.ProjectToPlane(local, plane);
            if (projected != null) local = projected;
            var planeToXY = Transform.PlaneToPlane(plane, Plane.WorldXY);
            local.Transform(planeToXY);
            return local;
        }

        private static int GetOrCreateLayer(RhinoDoc doc, string name, System.Drawing.Color color)
        {
            int idx = doc.Layers.FindByFullPath(name, -1);
            if (idx < 0)
            {
                var l = new Layer { Name = name, Color = color };
                idx = doc.Layers.Add(l);
            }
            return idx;
        }

        private static void Flush(List<string> lines)
        {
            foreach (var line in lines)
                RhinoApp.WriteLine(line);
            try
            {
                string path = Path.Combine(
                    System.Environment.GetFolderPath(
                        System.Environment.SpecialFolder.DesktopDirectory),
                    "phase30_frames.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase30_frames.txt: {ex.Message}");
            }
        }
    }
}
