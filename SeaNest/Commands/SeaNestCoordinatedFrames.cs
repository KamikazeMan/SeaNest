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
                var go = new GetObject();
                go.SetCommandPrompt("Select all parts to nest (frames + everything else)");
                go.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
                go.SubObjectSelect = false;
                go.GetMultiple(1, 0);
                if (go.CommandResult() != Result.Success)
                    return go.CommandResult();

                double sheetW = 240.0, sheetH = 72.0, spacing = 0.25, margin = 0.0;
                RhinoGet.GetNumber("Sheet width", false, ref sheetW);
                RhinoGet.GetNumber("Sheet height", false, ref sheetH);
                RhinoGet.GetNumber("Part spacing", false, ref spacing);
                diag.Add($"Sheet {sheetW:F2} x {sheetH:F2}, spacing {spacing:F3}, margin {margin:F2}");

                // Flatten selections to engine polygons (same path as the nest commands).
                var polygons = new List<Polygon>();
                for (int i = 0; i < go.ObjectCount; i++)
                {
                    var brep = ToBrep(go.Object(i));
                    if (brep == null) continue;
                    var flat = BrepFlattener.Flatten(brep, doc);
                    if (flat?.OuterPolygon != null)
                        polygons.Add(flat.OuterPolygon);
                }
                if (polygons.Count == 0)
                {
                    diag.Add("No parts could be flattened.");
                    Flush(diag);
                    return Result.Failure;
                }
                diag.Add($"Flattened {polygons.Count} part(s).");

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

        private static Brep ToBrep(ObjRef objRef)
        {
            if (objRef == null) return null;
            var brep = objRef.Brep();
            if (brep != null) return brep;
            if (objRef.Geometry() is Extrusion ext) return ext.ToBrep();
            return null;
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
