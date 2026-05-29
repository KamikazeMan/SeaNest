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

                // Topology partition (verbatim from ReNest): containedBy via
                // pairwise PlanarClosedCurveRelationship, immediate parent =
                // smallest container, outers = curves with no parent.
                var topologyPlane = Plane.WorldXY;
                double topologyTol = modelTol * 10.0;
                int nC = closedCurves.Count;
                var containedBy = new List<int>[nC];
                var bboxArea = new double[nC];
                for (int i = 0; i < nC; i++)
                {
                    containedBy[i] = new List<int>();
                    var bb = closedCurves[i].GetBoundingBox(true);
                    bboxArea[i] = bb.Diagonal.X * bb.Diagonal.Y;
                }
                for (int i = 0; i < nC - 1; i++)
                    for (int j = i + 1; j < nC; j++)
                    {
                        var rel = Curve.PlanarClosedCurveRelationship(
                            closedCurves[i], closedCurves[j], topologyPlane, topologyTol);
                        if (rel == RegionContainment.AInsideB) containedBy[i].Add(j);
                        else if (rel == RegionContainment.BInsideA) containedBy[j].Add(i);
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

                // Convert each outer to a Polygon (verbatim CurveToPolygon path).
                var polygons = new List<Polygon>();
                foreach (int outerIdx in outers)
                {
                    var outerCurve = closedCurves[outerIdx];
                    if (!outerCurve.TryGetPlane(out Plane op, discretizeTol * 10.0))
                        op = Plane.WorldXY;
                    var converted = CurveToPolygon(outerCurve, op, discretizeTol, angleTolRad, MaxVertices);
                    if (converted == null) continue;
                    polygons.Add(converted.Value.polygon);
                }
                if (polygons.Count == 0)
                {
                    diag.Add("No curves could be converted. Aborting.");
                    Flush(diag);
                    return Result.Failure;
                }
                diag.Add($"Read {polygons.Count} outer part(s) from the nested layout " +
                         $"({nC} closed curve(s); {nC - outers.Count} hole(s) excluded).");

                var request = new NestRequest(
                    polygons, sheetW, sheetH, 0.0, margin, spacing,
                    RotationStep.Every90, NestingAlgorithm.NFP_Annealed,
                    allowMirror: false, TimeSpan.FromSeconds(60));

                OrientedPart.BuildAll(
                    polygons, request.RotationStepDegrees, request.AllowMirror,
                    out _, out var orientationsByPart);
                var cache = new NfpCache(request.Spacing);
                var engine = new NfpPlacementEngine(request, orientationsByPart, cache);

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

                var result = engine.PlaceCoordinatedFrames(frames, remaining, diag);

                // Focused validation: no overlap among placed frames.
                var framePlacements = result.Placements
                    .Where(p => frameSet.Contains(p.OriginalIndex))
                    .ToList();
                int frameOverlaps = 0;
                for (int a = 0; a < framePlacements.Count; a++)
                    for (int b = a + 1; b < framePlacements.Count; b++)
                        if (OverlapChecker.Overlaps(
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
                    doc.Objects.AddCurve(
                        PolygonToCurve.ToCurve(p.PlacedPolygon, yOff),
                        new ObjectAttributes { LayerIndex = lyr });
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
