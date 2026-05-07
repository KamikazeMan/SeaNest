using System;
using System.Collections.Generic;
using System.Linq;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Input.Custom;

namespace SeaNest.Commands
{
    public class SeaNestTrimCommand : Command
    {
        public override string EnglishName => "SeaNestTrim";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            try { return RunInner(doc); }
            catch (Exception ex) { RhinoApp.WriteLine("Crash: " + ex.Message); return Result.Failure; }
        }

        private Result RunInner(RhinoDoc doc)
        {
            var go = new GetObject();
            go.SetCommandPrompt("Select intersecting breps to auto-trim");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go.EnablePreSelect(true, true);
            go.GetMultiple(2, 0);

            if (go.CommandResult() != Result.Success) return go.CommandResult();

            var breps = new List<Brep>();
            var brepIds = new List<Guid>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var b = go.Object(i).Brep();
                if (b != null && b.IsValid)
                {
                    breps.Add(b.DuplicateBrep());
                    brepIds.Add(go.Object(i).ObjectId);
                }
            }

            if (breps.Count < 2) return Result.Failure;

            int layerIdx = GetOrCreateLayer(doc, "SeaNest_Trimmed", System.Drawing.Color.FromArgb(0, 200, 255));

            // Find intersection curves
            var curvesPerBrep = new Dictionary<int, List<Curve>>();

            for (int i = 0; i < breps.Count; i++)
            {
                for (int j = i + 1; j < breps.Count; j++)
                {
                    try
                    {
                        if (Intersection.BrepBrep(breps[i], breps[j], doc.ModelAbsoluteTolerance,
                            out Curve[] curves, out Point3d[] pts))
                        {
                            if (curves != null && curves.Length > 0)
                            {
                                if (!curvesPerBrep.ContainsKey(i)) curvesPerBrep[i] = new List<Curve>();
                                if (!curvesPerBrep.ContainsKey(j)) curvesPerBrep[j] = new List<Curve>();
                                curvesPerBrep[i].AddRange(curves);
                                curvesPerBrep[j].AddRange(curves);
                            }
                        }
                    }
                    catch { }
                }
            }

            if (curvesPerBrep.Count == 0) { RhinoApp.WriteLine("No intersections."); return Result.Success; }

            var keepPieces = new List<Brep>();
            var hideIds = new List<Guid>();

            for (int i = 0; i < breps.Count; i++)
            {
                if (!curvesPerBrep.ContainsKey(i)) continue;

                var brep = breps[i];
                var curves = curvesPerBrep[i].Where(c => c != null && c.IsValid).ToArray();
                if (curves.Length == 0) continue;

                // Boolean difference: subtract all OTHER breps from this one
                var otherBreps = new List<Brep>();
                for (int k = 0; k < breps.Count; k++)
                {
                    if (k != i) otherBreps.Add(breps[k]);
                }

                Brep[] boolResult = null;
                try
                {
                    boolResult = Brep.CreateBooleanDifference(
                        new[] { brep },
                        otherBreps.ToArray(),
                        doc.ModelAbsoluteTolerance);
                }
                catch (Exception ex)
                {
                    RhinoApp.WriteLine(string.Format("  Brep {0} boolean difference failed: {1}", i + 1, ex.Message));
                    continue;
                }

                if (boolResult == null || boolResult.Length == 0)
                {
                    RhinoApp.WriteLine(string.Format("  Brep {0}: boolean produced no result", i + 1));
                    continue;
                }

                RhinoApp.WriteLine(string.Format("  Brep {0}: boolean difference produced {1} piece(s)",
                    i + 1, boolResult.Length));

                // Keep the largest result
                var result = boolResult.OrderByDescending(b => GetSize(b)).First();
                if (result != null && result.IsValid)
                {
                    keepPieces.Add(result);
                    hideIds.Add(brepIds[i]);
                }
            }

            // Commit
            var attr = new ObjectAttributes { LayerIndex = layerIdx };
            int added = 0;
            foreach (var piece in keepPieces)
            {
                try
                {
                    if (piece != null && piece.IsValid)
                    {
                        if (doc.Objects.AddBrep(piece, attr) != Guid.Empty) added++;
                    }
                }
                catch { }
            }

            foreach (var id in hideIds)
            {
                try { doc.Objects.Delete(id, true); } catch { }
            }

            RhinoApp.WriteLine(string.Format("Trim complete: {0} pieces added.", added));
            try { doc.Views.Redraw(); } catch { }
            return Result.Success;
        }

        private Brep TryAddCaps(Brep piece, Curve[] cutCurves, RhinoDoc doc, int brepNum)
        {
            try
            {
                var nakedEdges = piece.DuplicateNakedEdgeCurves(true, false);
                if (nakedEdges == null || nakedEdges.Length == 0) return piece;

                RhinoApp.WriteLine(string.Format("  Brep {0}: {1} total naked edges", brepNum, nakedEdges.Length));

                // Filter to only edges that lie on a cut curve
                var cutEdges = new List<Curve>();
                double tolCheck = doc.ModelAbsoluteTolerance * 10;

                foreach (var ne in nakedEdges)
                {
                    if (ne == null) continue;
                    var testPt = ne.PointAtNormalizedLength(0.5);

                    foreach (var cc in cutCurves)
                    {
                        if (cc == null) continue;
                        if (cc.ClosestPoint(testPt, out double t))
                        {
                            if (cc.PointAt(t).DistanceTo(testPt) < tolCheck)
                            {
                                cutEdges.Add(ne);
                                break;
                            }
                        }
                    }
                }

                RhinoApp.WriteLine(string.Format("    Filtered to {0} cut edges", cutEdges.Count));
                if (cutEdges.Count == 0) return piece;

                var loops = Curve.JoinCurves(cutEdges.ToArray(), doc.ModelAbsoluteTolerance * 10);
                if (loops == null || loops.Length == 0) return piece;

                var caps = new List<Brep>();
                foreach (var loop in loops)
                {
                    if (loop == null || !loop.IsValid || !loop.IsClosed) continue;

                    Brep[] capResult = null;
                    try
                    {
                        if (loop.IsPlanar())
                        {
                            capResult = Brep.CreatePlanarBreps(loop, doc.ModelAbsoluteTolerance);
                        }
                        else
                        {
                            var patch = Brep.CreatePatch(new[] { loop }, 10, 10, doc.ModelAbsoluteTolerance);
                            if (patch != null && patch.IsValid)
                                capResult = new[] { patch };
                        }
                    }
                    catch { }

                    if (capResult != null) caps.AddRange(capResult);
                }

                if (caps.Count == 0) return piece;
                RhinoApp.WriteLine(string.Format("    Built {0} cap surface(s)", caps.Count));

                // Try to merge into single brep
                Brep[] merged = null;
                try
                {
                    var all = new List<Brep> { piece };
                    all.AddRange(caps);
                    merged = Brep.JoinBreps(all.ToArray(), doc.ModelAbsoluteTolerance * 10);
                }
                catch { }

                if (merged != null && merged.Length > 0 && merged[0] != null && merged[0].IsValid)
                {
                    RhinoApp.WriteLine(string.Format("  Brep {0}: capped and joined into {1} piece(s)",
                        brepNum, merged.Length));
                    return merged[0];
                }

                RhinoApp.WriteLine(string.Format("  Brep {0}: JoinBreps returned {1} pieces (expected 1)",
                    brepNum, merged?.Length ?? 0));

                RhinoApp.WriteLine(string.Format("  Brep {0}: could not join caps, keeping trimmed piece only", brepNum));
                return piece;
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("  Cap error: " + ex.Message);
                return piece;
            }
        }

        private int GetOrCreateLayer(RhinoDoc doc, string name, System.Drawing.Color color)
        {
            int idx = doc.Layers.FindByFullPath(name, -1);
            if (idx < 0) { var l = new Layer { Name = name, Color = color }; idx = doc.Layers.Add(l); }
            return idx;
        }

        private double GetSize(Brep brep)
        {
            try { var vp = VolumeMassProperties.Compute(brep); if (vp != null && vp.Volume > 0) return vp.Volume; } catch { }
            try { var ap = AreaMassProperties.Compute(brep); if (ap != null) return ap.Area; } catch { }
            try { return brep.GetBoundingBox(true).Diagonal.Length; } catch { }
            return 0;
        }
    }
}
