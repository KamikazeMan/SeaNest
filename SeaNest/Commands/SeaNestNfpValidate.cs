using System;
using System.Collections.Generic;
using System.IO;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input.Custom;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Nesting;
using SeaNest.RhinoAdapters;

namespace SeaNest.Commands
{
    // Phase 29 NFP-foundation validation (THROWAWAY spike, no CP-SAT).
    //
    // Validates that SeaNest's EXISTING NFP machinery can correctly compute
    // and visualize the no-fit polygon of two (possibly concave) hull frames
    // before any CP-SAT constraint encoding is attempted. Select two parts:
    // A (fixed, selected first) and B (moving, selected second). The command
    // reuses the exact Rhino->Polygon path the nest commands use
    // (BrepFlattener.Flatten -> .OuterPolygon) and the real NFP entry point
    // (NoFitPolygon.Compute), draws A, B, and the NFP on named layers, and
    // writes Desktop\phase29_nfp_validate.txt with the key result: is the NFP
    // outer contour CONVEX or CONCAVE (that determines CP-SAT difficulty).
    //
    // Pure NFP/geometry — uses no OR-Tools — so it builds on both net7.0 and
    // net48; no #if NET7_0 guard is needed.
    public class SeaNestNfpValidate : Command
    {
        public override string EnglishName => "SeaNestNfpValidate";

        // Pure geometric NFP (no part-to-part gap) so the convex/concave
        // result reflects the raw geometry, not an inflation artifact.
        private const double NfpSpacing = 0.0;

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var lines = new List<string>
            {
                $"Phase 29 NFP validation run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                ""
            };

            try
            {
                // --- Select two parts: A (fixed) then B (moving). ---
                var go = new GetObject();
                go.SetCommandPrompt("Select TWO parts: A (fixed) first, then B (moving)");
                go.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
                go.SubObjectSelect = false;
                go.GetMultiple(2, 2);
                if (go.CommandResult() != Result.Success)
                    return go.CommandResult();

                Brep brepA = ToBrep(go.Object(0));
                Brep brepB = ToBrep(go.Object(1));
                if (brepA == null || brepB == null)
                {
                    lines.Add("Could not obtain Brep geometry for one or both selections.");
                    FlushAndEcho(lines);
                    return Result.Failure;
                }

                // --- Reuse the EXACT nest-command conversion path. ---
                var flatA = BrepFlattener.Flatten(brepA, doc);
                var flatB = BrepFlattener.Flatten(brepB, doc);
                if (flatA?.OuterPolygon == null || flatB?.OuterPolygon == null)
                {
                    lines.Add("BrepFlattener returned no OuterPolygon for one or both parts.");
                    FlushAndEcho(lines);
                    return Result.Failure;
                }

                Polygon polyA = flatA.OuterPolygon;
                Polygon polyB = flatB.OuterPolygon;

                lines.Add($"Part A: {polyA.Count} verts, bbox " +
                          $"[{polyA.BoundingBox.MinX:F2},{polyA.BoundingBox.MinY:F2}]..." +
                          $"[{polyA.BoundingBox.MaxX:F2},{polyA.BoundingBox.MaxY:F2}], " +
                          $"{(polyA.IsConvex ? "convex" : "concave")}");
                lines.Add($"Part B: {polyB.Count} verts, bbox " +
                          $"[{polyB.BoundingBox.MinX:F2},{polyB.BoundingBox.MinY:F2}]..." +
                          $"[{polyB.BoundingBox.MaxX:F2},{polyB.BoundingBox.MaxY:F2}], " +
                          $"{(polyB.IsConvex ? "convex" : "concave")}");
                lines.Add("");

                // --- The real NFP computation (B relative to A). ---
                IReadOnlyList<Polygon> nfp =
                    NoFitPolygon.Compute(polyA, polyB, NfpSpacing);

                lines.Add($"NFP spacing: {NfpSpacing:F3}");
                lines.Add($"NFP contour count: {nfp.Count}");

                if (nfp.Count == 0)
                {
                    lines.Add("DEGENERACY: NFP is empty — parts do not interact " +
                              "(or inflation collapsed the geometry).");
                    FlushAndEcho(lines);
                    return Result.Success;
                }

                // Outer contours are CCW (Area > 0); holes are CW (Area < 0).
                // The canonical outer contour is the largest-area CCW polygon.
                Polygon outer = null;
                int outerCount = 0;
                int holeCount = 0;
                foreach (var p in nfp)
                {
                    if (p.IsCounterClockwise)
                    {
                        outerCount++;
                        if (outer == null || p.AbsoluteArea > outer.AbsoluteArea)
                            outer = p;
                    }
                    else
                    {
                        holeCount++;
                    }
                }

                lines.Add($"  outer (CCW) contours: {outerCount}, holes (CW): {holeCount}");

                if (outer == null)
                {
                    lines.Add("DEGENERACY: no CCW outer contour found (all contours CW). " +
                              "Reporting first contour's stats instead.");
                    outer = nfp[0];
                }

                var ob = outer.BoundingBox;
                lines.Add($"NFP outer contour: {outer.Count} verts, bbox " +
                          $"[{ob.MinX:F2},{ob.MinY:F2}]...[{ob.MaxX:F2},{ob.MaxY:F2}] " +
                          $"({ob.Width:F2} x {ob.Height:F2})");

                // THE KEY RESULT.
                bool nfpConvex = outer.IsConvex;
                lines.Add("");
                lines.Add($"=== KEY RESULT: NFP outer contour is {(nfpConvex ? "CONVEX" : "CONCAVE")} ===");
                lines.Add(nfpConvex
                    ? "  Convex NFP — relative placement can be encoded as a single "
                      + "intersection of half-planes (CP-SAT-friendly)."
                    : "  Concave NFP — relative placement requires a DISJUNCTION of "
                      + "half-plane regions (one per concave edge), i.e. big-M/reified "
                      + "boolean encoding in CP-SAT. This is the expected hard case.");

                // --- Draw into the doc on named layers. ---
                int layA = GetOrCreateLayer(doc, "NFP_A", System.Drawing.Color.SteelBlue);
                int layB = GetOrCreateLayer(doc, "NFP_B", System.Drawing.Color.SeaGreen);
                int layCurve = GetOrCreateLayer(doc, "NFP_curve", System.Drawing.Color.OrangeRed);
                int layHoles = GetOrCreateLayer(doc, "NFP_holes", System.Drawing.Color.Gray);

                doc.Objects.AddCurve(PolygonToCurve.ToCurve(polyA),
                    new ObjectAttributes { LayerIndex = layA });
                doc.Objects.AddCurve(PolygonToCurve.ToCurve(polyB),
                    new ObjectAttributes { LayerIndex = layB });

                foreach (var p in nfp)
                {
                    int idx = p.IsCounterClockwise ? layCurve : layHoles;
                    doc.Objects.AddCurve(PolygonToCurve.ToCurve(p),
                        new ObjectAttributes { LayerIndex = idx });
                }

                doc.Views.Redraw();
                lines.Add("");
                lines.Add("Drew A on NFP_A, B on NFP_B, NFP outer(s) on NFP_curve, holes on NFP_holes.");

                FlushAndEcho(lines);
                return Result.Success;
            }
            catch (Exception ex)
            {
                lines.Add($"EXCEPTION ({ex.GetType().Name}): {ex.Message}");
                if (ex.InnerException != null)
                    lines.Add($"  Inner ({ex.InnerException.GetType().Name}): {ex.InnerException.Message}");
                lines.Add(ex.StackTrace ?? "(no stack trace)");
                FlushAndEcho(lines);
                RhinoApp.WriteLine("SeaNestNfpValidate failed — see phase29_nfp_validate.txt.");
                return Result.Failure;
            }
        }

        private static Brep ToBrep(ObjRef objRef)
        {
            if (objRef == null) return null;
            var brep = objRef.Brep();
            if (brep != null) return brep;
            if (objRef.Geometry() is Extrusion ext)
                return ext.ToBrep();
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

        private static void FlushAndEcho(List<string> lines)
        {
            foreach (var line in lines)
                RhinoApp.WriteLine(line);

            try
            {
                string path = Path.Combine(
                    System.Environment.GetFolderPath(
                        System.Environment.SpecialFolder.DesktopDirectory),
                    "phase29_nfp_validate.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase29_nfp_validate.txt: {ex.Message}");
            }
        }
    }
}
