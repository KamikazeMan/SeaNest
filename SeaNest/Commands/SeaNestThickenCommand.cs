using System;
using System.Collections.Generic;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;

namespace SeaNest.Commands
{
    public class SeaNestThickenCommand : Command
    {
        public override string EnglishName => "SeaNestThicken";

        // Material Database
        private static readonly Dictionary<string, MaterialInfo> Materials = new Dictionary<string, MaterialInfo>
        {
            { "5083 Aluminum", new MaterialInfo(2.66, 5.0, 0.25) },
            { "Mild Steel", new MaterialInfo(7.85, 6.0, 0.25) },
            { "316 Stainless", new MaterialInfo(8.0, 4.0, 0.1875) },
            { "6061 Aluminum", new MaterialInfo(2.70, 5.0, 0.25) },
            { "Custom", new MaterialInfo(2.5, 3.0, 0.125) },
        };

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var settings = SeaNestPlugin.Instance.UserSettings;

            // STEP 1: Select surfaces
            var go = new GetObject();
            go.SetCommandPrompt("Select surfaces to thicken");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go.EnablePreSelect(true, true);
            go.GetMultiple(1, 0);
            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            var selectedObjects = new List<ObjRef>();
            for (int i = 0; i < go.ObjectCount; i++)
                selectedObjects.Add(go.Object(i));

            RhinoApp.WriteLine("SeaNest Thicken: {0} surface(s) selected.", selectedObjects.Count);

            // STEP 2: Material
            var gm = new GetOption();
            gm.SetCommandPrompt("Select material");
            int idx5083 = gm.AddOption("5083_Aluminum");
            int idxMild = gm.AddOption("Mild_Steel");
            int idx316 = gm.AddOption("316_Stainless");
            int idx6061 = gm.AddOption("6061_Aluminum");
            int idxCustom = gm.AddOption("Custom");
            gm.SetDefaultString("5083_Aluminum");
            gm.AcceptNothing(true);

            var gmResult = gm.Get();
            string materialName = "5083 Aluminum";
            double customDensity = 2.5;

            if (gmResult == GetResult.Option)
            {
                var opt = gm.Option();
                if (opt.Index == idx5083) materialName = "5083 Aluminum";
                else if (opt.Index == idxMild) materialName = "Mild Steel";
                else if (opt.Index == idx316) materialName = "316 Stainless";
                else if (opt.Index == idx6061) materialName = "6061 Aluminum";
                else if (opt.Index == idxCustom)
                {
                    materialName = "Custom";
                    var gd = new GetNumber();
                    gd.SetCommandPrompt("Custom density (g/cm³)");
                    gd.SetDefaultNumber(2.5);
                    gd.SetLowerLimit(0.1, false);
                    gd.SetUpperLimit(25.0, false);
                    if (gd.Get() == GetResult.Number)
                        customDensity = gd.Number();
                }
            }

            var material = Materials[materialName];
            double density = materialName == "Custom" ? customDensity : material.Density;
            RhinoApp.WriteLine("SeaNest: Material = {0}, Density = {1} g/cm³", materialName, density);

            // STEP 3: Thickness
            bool useInches = doc.ModelUnitSystem == Rhino.UnitSystem.Inches ||
                             doc.ModelUnitSystem == Rhino.UnitSystem.Feet;
            double defaultThickness = useInches ? material.DefaultThicknessIn : material.DefaultThicknessMm;
            string unitLabel = useInches ? "inches" : "mm";

            double thickness = defaultThickness;
            bool gotThickness = false;
            while (!gotThickness)
            {
                var gt = new GetNumber();
                gt.SetCommandPrompt(string.Format("Thickness in {0}", unitLabel));
                gt.SetDefaultNumber(thickness);
                gt.SetLowerLimit(0.001, false);
                gt.AcceptNothing(true);
                gt.AddOption("Preset_1_8_inch");
                gt.AddOption("Preset_1_4_inch");
                gt.AddOption("Preset_3_16_inch");
                gt.AddOption("Preset_5mm");
                gt.AddOption("Preset_6mm");

                var gtResult = gt.Get();
                if (gtResult == GetResult.Number) { thickness = gt.Number(); gotThickness = true; }
                else if (gtResult == GetResult.Option)
                {
                    var optName = gt.Option().EnglishName;
                    switch (optName)
                    {
                        case "Preset_1_8_inch": thickness = useInches ? 0.125 : 3.175; break;
                        case "Preset_1_4_inch": thickness = useInches ? 0.25 : 6.35; break;
                        case "Preset_3_16_inch": thickness = useInches ? 0.1875 : 4.7625; break;
                        case "Preset_5mm": thickness = useInches ? 0.19685 : 5.0; break;
                        case "Preset_6mm": thickness = useInches ? 0.23622 : 6.0; break;
                    }
                    gotThickness = true;
                }
                else if (gtResult == GetResult.Nothing) gotThickness = true;
                else return Result.Cancel;
            }

            RhinoApp.WriteLine("SeaNest: Thickness = {0} {1}", thickness, unitLabel);

            // STEP 4: Direction
            var gdir = new GetOption();
            gdir.SetCommandPrompt("Thicken direction");
            int dirBoth = gdir.AddOption("Both_Split");
            int dirOut = gdir.AddOption("Outward");
            int dirIn = gdir.AddOption("Inward");
            gdir.SetDefaultString("Both_Split");
            gdir.AcceptNothing(true);

            string direction = "both";
            var gdirResult = gdir.Get();
            if (gdirResult == GetResult.Option)
            {
                var opt = gdir.Option();
                if (opt.Index == dirOut) direction = "outward";
                else if (opt.Index == dirIn) direction = "inward";
                else direction = "both";
            }

            RhinoApp.WriteLine("SeaNest: Direction = {0}", direction);

            // STEP 5: Options
            var gopts = new GetOption();
            gopts.SetCommandPrompt("Options (Enter to proceed)");
            var hideOrigOpt = new OptionToggle(true, "No", "Yes");
            var assignMatOpt = new OptionToggle(true, "No", "Yes");
            gopts.AddOptionToggle("HideOriginals", ref hideOrigOpt);
            gopts.AddOptionToggle("AssignMaterial", ref assignMatOpt);
            gopts.AcceptNothing(true);
            while (true)
            {
                var optResult = gopts.Get();
                if (optResult == GetResult.Nothing || optResult == GetResult.Cancel) break;
            }
            bool hideOriginals = hideOrigOpt.CurrentValue;
            bool assignMat = assignMatOpt.CurrentValue;

            // STEP 6: Process
            RhinoApp.WriteLine("");
            RhinoApp.WriteLine("=== SeaNest Thickening Start ===");

            int successCount = 0;
            int failCount = 0;
            double totalVolume = 0.0;
            double totalWeight = 0.0;
            var newSolidIds = new List<Guid>();

            string layerName = "SeaNest_Thickened";
            int layerIndex = doc.Layers.FindByFullPath(layerName, -1);
            if (layerIndex < 0)
            {
                var layer = new Layer { Name = layerName, Color = System.Drawing.Color.FromArgb(0, 255, 100) };
                layerIndex = doc.Layers.Add(layer);
            }

            foreach (var objRef in selectedObjects)
            {
                Brep brep = TryExtractBrep(objRef);
                if (brep == null || !brep.IsValid)
                {
                    RhinoApp.WriteLine("  Skipping object — not a valid surface/brep.");
                    failCount++;
                    continue;
                }

                RhinoApp.WriteLine("  Processing surface {0} of {1}...", successCount + failCount + 1, selectedObjects.Count);

                Brep solidBrep = ThickenBrep(brep, thickness, direction, doc);

                if (solidBrep != null)
                {
                    var attr = new ObjectAttributes
                    {
                        LayerIndex = layerIndex,
                        ColorSource = ObjectColorSource.ColorFromObject,
                        ObjectColor = System.Drawing.Color.FromArgb(0, 255, 100),
                    };
                    var origObj = objRef.Object();
                    string partName = origObj?.Name;
                    if (string.IsNullOrEmpty(partName))
                        partName = "Part_" + (successCount + 1).ToString("D3");
                    attr.Name = partName + "_solid";

                    var newId = doc.Objects.AddBrep(solidBrep, attr);
                    if (newId != Guid.Empty)
                    {
                        newSolidIds.Add(newId);
                        if (assignMat)
                        {
                            var newObj = doc.Objects.Find(newId);
                            if (newObj != null)
                            {
                                newObj.Attributes.SetUserString("SeaNest_Material", materialName);
                                newObj.Attributes.SetUserString("SeaNest_Density", density.ToString("F2"));
                                newObj.Attributes.SetUserString("SeaNest_Thickness", thickness.ToString("F4") + " " + unitLabel);
                                newObj.Attributes.SetUserString("SeaNest_Direction", direction);
                                newObj.Attributes.SetUserString("SeaNest_ProcessedBy", "SeaNest v1.0");
                                newObj.CommitChanges();
                            }
                        }

                        var mp = VolumeMassProperties.Compute(solidBrep);
                        if (mp != null)
                        {
                            double volume = mp.Volume;
                            totalVolume += volume;
                            double volumeCm3 = volume;
                            if (doc.ModelUnitSystem == Rhino.UnitSystem.Inches) volumeCm3 = volume * 16.387064;
                            else if (doc.ModelUnitSystem == Rhino.UnitSystem.Feet) volumeCm3 = volume * 28316.846592;
                            else if (doc.ModelUnitSystem == Rhino.UnitSystem.Meters) volumeCm3 = volume * 1000000.0;
                            double weightG = volumeCm3 * density;
                            double weightKg = weightG / 1000.0;
                            totalWeight += weightKg;
                            RhinoApp.WriteLine("    Volume: {0:F2} {1}³ | Weight: {2:F3} kg", volume, unitLabel, weightKg);
                        }

                        successCount++;
                        RhinoApp.WriteLine("    SUCCESS — solid created.");
                    }
                    else
                    {
                        failCount++;
                        RhinoApp.WriteLine("    FAILED — could not add solid to document.");
                    }
                }
                else
                {
                    failCount++;
                    RhinoApp.WriteLine("    FAILED — could not create offset solid.");
                    RhinoApp.WriteLine("    Tip: Check that the surface is valid and not too complex.");
                }
            }

            // STEP 7: Hide originals
            if (hideOriginals && successCount > 0)
            {
                foreach (var objRef in selectedObjects)
                    doc.Objects.Hide(objRef.ObjectId, true);
                RhinoApp.WriteLine("  Original surfaces hidden.");
            }

            // STEP 8: COG
            Point3d cog = Point3d.Origin;
            if (newSolidIds.Count > 0)
            {
                double totalMass = 0;
                var weightedSum = new Vector3d(0, 0, 0);
                foreach (var id in newSolidIds)
                {
                    var obj = doc.Objects.Find(id);
                    if (obj == null) continue;
                    var brep2 = (obj.Geometry as Brep);
                    if (brep2 == null) continue;
                    var mp = VolumeMassProperties.Compute(brep2);
                    if (mp == null) continue;
                    double vol = mp.Volume;
                    var centroid = mp.Centroid;
                    weightedSum += new Vector3d(centroid.X * vol, centroid.Y * vol, centroid.Z * vol);
                    totalMass += vol;
                }
                if (totalMass > 0)
                    cog = new Point3d(weightedSum.X / totalMass, weightedSum.Y / totalMass, weightedSum.Z / totalMass);
            }

            // STEP 9: Report
            RhinoApp.WriteLine("");
            RhinoApp.WriteLine("==========================================");
            RhinoApp.WriteLine("  SeaNest Thickening Complete");
            RhinoApp.WriteLine("==========================================");
            RhinoApp.WriteLine("  Processed: {0} of {1} surfaces", successCount, selectedObjects.Count);
            if (failCount > 0)
                RhinoApp.WriteLine("  Failed: {0}", failCount);
            RhinoApp.WriteLine("  Material: {0} ({1} g/cm³)", materialName, density);
            RhinoApp.WriteLine("  Thickness: {0} {1} | Direction: {2}", thickness, unitLabel, direction);

            if (useInches)
            {
                double weightLbs = totalWeight * 2.20462;
                RhinoApp.WriteLine("  Total Volume: {0:F3} in³", totalVolume);
                RhinoApp.WriteLine("  Total Weight: {0:F2} lbs ({1:F3} kg)", weightLbs, totalWeight);
            }
            else
            {
                RhinoApp.WriteLine("  Total Volume: {0:F2} cm³", totalVolume);
                RhinoApp.WriteLine("  Total Weight: {0:F3} kg ({1:F2} lbs)", totalWeight, totalWeight * 2.20462);
            }

            if (newSolidIds.Count > 0)
                RhinoApp.WriteLine("  Center of Gravity: ({0:F2}, {1:F2}, {2:F2})", cog.X, cog.Y, cog.Z);

            RhinoApp.WriteLine("==========================================");
            RhinoApp.WriteLine("  Solids on layer: SeaNest_Thickened");
            RhinoApp.WriteLine("==========================================");

            doc.Views.Redraw();
            return Result.Success;
        }

        // ------------------------------------------------------------------
        // Brep extraction — multi-fallback
        // ------------------------------------------------------------------

        private static Brep TryExtractBrep(ObjRef objRef)
        {
            try
            {
                var b = objRef.Brep();
                if (b != null) return b;
            }
            catch { }

            try
            {
                var srf = objRef.Surface();
                if (srf != null) return Brep.CreateFromSurface(srf);
            }
            catch { }

            try
            {
                var geo = objRef.Geometry();
                if (geo is Surface surf) return Brep.CreateFromSurface(surf);
                if (geo is Brep b) return b.DuplicateBrep();
            }
            catch { }

            return null;
        }

        // ------------------------------------------------------------------
        // Thickening: dispatcher with progressively-more-fallback strategies
        //
        // Strategy order:
        //   1. Planar face → curve extrusion (clean corners, fastest, most reliable for flat parts)
        //   2. Single-shot CreateOffsetBrep with solid=true (works on simple curved surfaces)
        //   3. Manual offset+loft+cap (the OffsetSrf-equivalent for tricky Patches)
        //   4. _OffsetSrf command via RunScript (last-resort, uses Rhino's internal command)
        // ------------------------------------------------------------------

        private static Brep ThickenBrep(Brep brep, double thickness, string direction, RhinoDoc doc)
        {
            double tol = doc.ModelAbsoluteTolerance;

            // Strategy 1: Planar
            try
            {
                if (brep.Faces.Count >= 1)
                {
                    var face = brep.Faces[0];
                    if (face.IsPlanar(tol) && brep.Edges.Count > 0)
                    {
                        var planarSolid = ThickenPlanarBrep(brep, face, thickness, direction, tol);
                        if (planarSolid != null && planarSolid.IsValid)
                            return planarSolid;
                    }
                }
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("    Planar strategy threw: {0}", ex.Message);
            }

            // Curved branch — try strategies 2, 3, 4 in order.
            RhinoApp.WriteLine("    Curved surface — trying offset strategies...");

            double offsetDist;
            switch (direction)
            {
                case "inward": offsetDist = -thickness; break;
                case "outward": offsetDist = thickness; break;
                default: offsetDist = thickness; break; // "both" handled inside strategies
            }

            // Strategy 2: Single-shot CreateOffsetBrep
            try
            {
                var oneShot = TryOneShotOffset(brep, thickness, direction, tol);
                if (oneShot != null && oneShot.IsValid && oneShot.IsSolid)
                {
                    RhinoApp.WriteLine("    Used single-shot CreateOffsetBrep.");
                    return oneShot;
                }
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("    Single-shot offset threw: {0}", ex.Message);
            }

            // Strategy 3: Manual offset + loft + cap
            try
            {
                var manual = TryManualThicken(brep, thickness, direction, tol);
                if (manual != null && manual.IsValid)
                {
                    RhinoApp.WriteLine("    Used manual offset+loft+cap.");
                    return manual;
                }
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("    Manual thicken threw: {0}", ex.Message);
            }

            // Strategy 4: _OffsetSrf command fallback
            try
            {
                var cmdResult = TryRhinoCommandOffset(brep, thickness, direction, doc);
                if (cmdResult != null && cmdResult.IsValid)
                {
                    RhinoApp.WriteLine("    Used _OffsetSrf command fallback.");
                    return cmdResult;
                }
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine("    Command fallback threw: {0}", ex.Message);
            }

            return null;
        }

        // ------------------------------------------------------------------
        // Strategy 1: Planar curve extrusion
        // ------------------------------------------------------------------

        private static Brep ThickenPlanarBrep(Brep brep, BrepFace face, double thickness, string direction, double tol)
        {
            var curves = new List<Curve>();
            foreach (var edge in brep.Edges)
                curves.Add(edge.DuplicateCurve());

            var joined = Curve.JoinCurves(curves, tol);
            if (joined == null || joined.Length == 0) return null;

            var profile = joined[0];
            double uMid = face.Domain(0).Mid;
            double vMid = face.Domain(1).Mid;
            var normal = face.NormalAt(uMid, vMid);
            normal.Unitize();

            if (direction == "both")
            {
                var movedProfile = profile.DuplicateCurve();
                movedProfile.Translate(normal * (-thickness / 2.0));
                var ext = Extrusion.Create(movedProfile, thickness, true);
                return ext?.ToBrep();
            }
            else if (direction == "outward")
            {
                var ext = Extrusion.Create(profile, thickness, true);
                return ext?.ToBrep();
            }
            else
            {
                var ext = Extrusion.Create(profile, -thickness, true);
                return ext?.ToBrep();
            }
        }

        // ------------------------------------------------------------------
        // Strategy 2: Single-shot offset
        // ------------------------------------------------------------------

        private static Brep TryOneShotOffset(Brep brep, double thickness, string direction, double tol)
        {
            double offsetDist;
            switch (direction)
            {
                case "inward": offsetDist = -thickness; break;
                case "both": offsetDist = thickness / 2.0; break; // single-shot doesn't truly center, best-effort
                default: offsetDist = thickness; break;
            }

            var result = Brep.CreateOffsetBrep(brep, offsetDist, true, true, tol, out _, out _);
            if (result == null || result.Length == 0) return null;

            // Take the largest piece if multiple
            Brep best = result[0];
            double bestVol = VolumeMassProperties.Compute(best)?.Volume ?? 0;
            for (int i = 1; i < result.Length; i++)
            {
                double v = VolumeMassProperties.Compute(result[i])?.Volume ?? 0;
                if (v > bestVol) { best = result[i]; bestVol = v; }
            }
            return best;
        }

        // ------------------------------------------------------------------
        // Strategy 3: Manual offset + loft + cap (the OffsetSrf-equivalent)
        //
        // For each face:
        //   1. Compute the offset of the face's surface (no solid, no loose) — gives us an offset surface
        //   2. Build a Brep from the original and offset surfaces
        //   3. Loft side walls between corresponding boundary edges
        //   4. Join everything into a closed solid
        //
        // Centered ("both") direction is implemented by offsetting the original by -t/2 and +t/2
        // and lofting between THOSE two surfaces.
        // ------------------------------------------------------------------

        private static Brep TryManualThicken(Brep brep, double thickness, string direction, double tol)
        {
            // Choose the two boundary surfaces.
            Brep bottomShell;
            Brep topShell;

            if (direction == "both")
            {
                bottomShell = OffsetShell(brep, -thickness / 2.0, tol);
                topShell = OffsetShell(brep, thickness / 2.0, tol);
            }
            else if (direction == "inward")
            {
                bottomShell = OffsetShell(brep, -thickness, tol);
                topShell = brep.DuplicateBrep();
            }
            else // outward
            {
                bottomShell = brep.DuplicateBrep();
                topShell = OffsetShell(brep, thickness, tol);
            }

            if (bottomShell == null || topShell == null) return null;
            if (!bottomShell.IsValid || !topShell.IsValid) return null;

            // Get matching boundary loops (naked edges) from both shells.
            var bottomEdges = GetNakedEdgeCurves(bottomShell, tol);
            var topEdges = GetNakedEdgeCurves(topShell, tol);

            if (bottomEdges == null || topEdges == null) return null;
            if (bottomEdges.Count == 0 || topEdges.Count == 0) return null;
            if (bottomEdges.Count != topEdges.Count) return null;

            // Loft pairs of corresponding loops to make side walls.
            var pieces = new List<Brep> { bottomShell, topShell };

            for (int i = 0; i < bottomEdges.Count; i++)
            {
                var bottom = bottomEdges[i];
                var top = MatchClosestLoop(bottom, topEdges);
                if (top == null) continue;

                var loftCurves = new[] { bottom, top };
                var lofted = Brep.CreateFromLoft(
                    loftCurves,
                    Point3d.Unset, Point3d.Unset,
                    LoftType.Straight,
                    false);
                if (lofted == null || lofted.Length == 0) continue;
                pieces.AddRange(lofted);
            }

            // Join everything.
            var joined = Brep.JoinBreps(pieces, tol);
            if (joined == null || joined.Length == 0) return null;

            // Pick the closed solid result.
            foreach (var j in joined)
            {
                if (j != null && j.IsValid && j.IsSolid)
                    return j;
            }

            // No solid — try to cap any remaining open Breps.
            foreach (var j in joined)
            {
                if (j == null || !j.IsValid) continue;
                var capped = j.CapPlanarHoles(tol);
                if (capped != null && capped.IsValid && capped.IsSolid) return capped;
            }

            return null;
        }

        /// <summary>
        /// Offset a Brep's surface(s) without trying to build a solid. Returns the offset shell
        /// as an open Brep — much more reliable than CreateOffsetBrep with solid=true.
        /// </summary>
        private static Brep OffsetShell(Brep brep, double distance, double tol)
        {
            // CreateOffsetBrep with solid=false, extend=false: just an offset surface, no walls.
            var result = Brep.CreateOffsetBrep(brep, distance, false, false, tol, out _, out _);
            if (result == null || result.Length == 0) return null;

            if (result.Length == 1) return result[0];

            // If multiple, join them (a polysurface offset can produce multiple pieces).
            var joined = Brep.JoinBreps(result, tol);
            return joined != null && joined.Length > 0 ? joined[0] : result[0];
        }

        private static List<Curve> GetNakedEdgeCurves(Brep brep, double tol)
        {
            var edges = new List<Curve>();
            foreach (var edge in brep.Edges)
            {
                if (edge.Valence == EdgeAdjacency.Naked)
                {
                    var c = edge.DuplicateCurve();
                    if (c != null) edges.Add(c);
                }
            }
            if (edges.Count == 0) return null;

            var joined = Curve.JoinCurves(edges, tol);
            return joined != null ? new List<Curve>(joined) : edges;
        }

        /// <summary>
        /// Find the loop in <paramref name="candidates"/> whose midpoint is closest to
        /// <paramref name="reference"/>'s midpoint. Used to match corresponding boundary
        /// loops between two parallel shells.
        /// </summary>
        private static Curve MatchClosestLoop(Curve reference, List<Curve> candidates)
        {
            var refMid = reference.PointAtNormalizedLength(0.5);
            Curve best = null;
            double bestDist = double.MaxValue;
            foreach (var c in candidates)
            {
                var mid = c.PointAtNormalizedLength(0.5);
                double d = mid.DistanceTo(refMid);
                if (d < bestDist) { bestDist = d; best = c; }
            }
            return best;
        }

        // ------------------------------------------------------------------
        // Strategy 4: Rhino's _OffsetSrf command (last resort)
        // ------------------------------------------------------------------

        private static Brep TryRhinoCommandOffset(Brep brep, double thickness, string direction, RhinoDoc doc)
        {
            // Add the source temporarily, run the command, capture the result, clean up.
            var tempId = doc.Objects.AddBrep(brep);
            if (tempId == Guid.Empty) return null;

            try
            {
                doc.Objects.UnselectAll();
                doc.Objects.Select(tempId);

                double signedDist;
                switch (direction)
                {
                    case "inward": signedDist = -thickness; break;
                    case "both": signedDist = thickness; break; // OffsetSrf doesn't have "both", outward by default
                    default: signedDist = thickness; break;
                }

                string cmd = string.Format("_-OffsetSrf _Solid=Yes _BothSides=No _DeleteInput=No {0} _Enter",
                    signedDist.ToString("F6", System.Globalization.CultureInfo.InvariantCulture));
                RhinoApp.RunScript(cmd, false);

                Brep result = null;
                foreach (var sel in doc.Objects.GetSelectedObjects(false, false))
                {
                    if (sel.Id == tempId) continue;
                    var b = sel.Geometry as Brep;
                    if (b != null && b.IsValid)
                    {
                        result = b.DuplicateBrep();
                        doc.Objects.Delete(sel.Id, true);
                        break;
                    }
                }
                return result;
            }
            finally
            {
                doc.Objects.Delete(tempId, true);
                doc.Objects.UnselectAll();
            }
        }

        // ------------------------------------------------------------------
        private class MaterialInfo
        {
            public double Density { get; }
            public double DefaultThicknessMm { get; }
            public double DefaultThicknessIn { get; }
            public MaterialInfo(double density, double thicknessMm, double thicknessIn)
            {
                Density = density;
                DefaultThicknessMm = thicknessMm;
                DefaultThicknessIn = thicknessIn;
            }
        }
    }
}