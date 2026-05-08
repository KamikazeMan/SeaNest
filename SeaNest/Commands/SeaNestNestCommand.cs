using System;
using System.Collections.Generic;
using System.Linq;
using Eto.Forms;
using Rhino;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input;
using Rhino.Input.Custom;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Nesting;
using SeaNest.Nesting.Core.Verification;
using SeaNest.RhinoAdapters;
using RhinoCommand = Rhino.Commands.Command;

namespace SeaNest.Commands
{
    public class SeaNestNestCommand : RhinoCommand
    {
        public override string EnglishName => "SeaNestNest";

        private const string LayerNested = "SeaNest_Nested";
        private const string LayerSheets = "SeaNest_Sheets";
        private const string LayerLabels = "SeaNest_Labels";
        private const string DimStyleName = "SeaNest_Labels";

        private const double DefaultSheetWidthIn = 96.0;
        private const double DefaultSheetHeightIn = 48.0;
        private const double DefaultThicknessIn = 0.25;
        private const double DefaultMarginIn = 0.25;
        private const double DefaultSpacingIn = 0.25;
        private const double LabelHeightIn = 6.0;
        private const double LabelStrideFactor = 3.0;

        // Phase 2 defaults — keep BLF the default to preserve Phase 1 behavior.
        private const NestingAlgorithm DefaultAlgorithm = NestingAlgorithm.BLF;
        private const bool DefaultAllowMirror = true;
        private const double DefaultTimeBudgetSeconds = 30.0;

        protected override Rhino.Commands.Result RunCommand(RhinoDoc doc, Rhino.Commands.RunMode mode)
        {
            bool isMetric = doc.ModelUnitSystem == UnitSystem.Millimeters
                         || doc.ModelUnitSystem == UnitSystem.Centimeters
                         || doc.ModelUnitSystem == UnitSystem.Meters;

            double inToModel = isMetric
                ? RhinoMath.UnitScale(UnitSystem.Inches, doc.ModelUnitSystem)
                : 1.0;

            double sheetW, sheetH, sheetT, margin, spacing;
            RotationStep rotStep;
            NestingAlgorithm algorithm;
            bool allowMirror;
            double timeBudgetSeconds;

            if (!PromptForDouble(doc, "Sheet width", DefaultSheetWidthIn * inToModel, out sheetW)) return Rhino.Commands.Result.Cancel;
            if (!PromptForDouble(doc, "Sheet height", DefaultSheetHeightIn * inToModel, out sheetH)) return Rhino.Commands.Result.Cancel;
            if (!PromptForDouble(doc, "Sheet thickness", DefaultThicknessIn * inToModel, out sheetT)) return Rhino.Commands.Result.Cancel;
            if (!PromptForDouble(doc, "Margin", DefaultMarginIn * inToModel, out margin)) return Rhino.Commands.Result.Cancel;
            if (!PromptForDouble(doc, "Spacing", DefaultSpacingIn * inToModel, out spacing)) return Rhino.Commands.Result.Cancel;
            if (!PromptForRotation(out rotStep)) return Rhino.Commands.Result.Cancel;
            if (!PromptForAlgorithm(out algorithm)) return Rhino.Commands.Result.Cancel;

            // Mirror and TimeBudget are only meaningful for NFP paths. Skip the prompts
            // for BLF so the user's existing flow is untouched when running BLF.
            if (algorithm == NestingAlgorithm.BLF)
            {
                allowMirror = false;
                timeBudgetSeconds = DefaultTimeBudgetSeconds;
            }
            else
            {
                if (!PromptForBool("Allow part mirroring", DefaultAllowMirror, out allowMirror))
                    return Rhino.Commands.Result.Cancel;

                if (algorithm == NestingAlgorithm.NFP_Annealed)
                {
                    if (!PromptForDouble(doc, "Time budget (seconds)", DefaultTimeBudgetSeconds, out timeBudgetSeconds))
                        return Rhino.Commands.Result.Cancel;
                    if (timeBudgetSeconds <= 0)
                    {
                        RhinoApp.WriteLine("Time budget must be positive.");
                        return Rhino.Commands.Result.Cancel;
                    }
                }
                else
                {
                    timeBudgetSeconds = DefaultTimeBudgetSeconds;
                }
            }

            var go = new GetObject();
            go.SetCommandPrompt("Select parts to nest");
            go.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
            go.GroupSelect = true;
            go.SubObjectSelect = false;
            go.GetMultiple(1, 0);
            if (go.CommandResult() != Rhino.Commands.Result.Success)
                return go.CommandResult();

            var breps = new List<Brep>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var obj = go.Object(i);
                var brep = obj.Brep();
                if (brep == null)
                {
                    var ext = obj.Geometry() as Extrusion;
                    if (ext != null) brep = ext.ToBrep();
                }
                if (brep != null) breps.Add(brep);
            }

            if (breps.Count == 0)
            {
                RhinoApp.WriteLine("No valid Breps selected.");
                return Rhino.Commands.Result.Nothing;
            }

            // Wire the squish warning so users see which parts came through Squish
            // (and will therefore have approximate dimensions). Single writer:
            // surface every message to the user AND tally Squish hits in one place.
            int squishedCount = 0;
            BrepFlattener.SquishWarning = msg =>
            {
                if (msg == null) return;
                RhinoApp.WriteLine(msg);
                if (msg.Contains("Squish")) squishedCount++;
            };

            // Wire the simplify warning so users see which parts had their pre-NFP
            // simplification escalated past the default tolerance because the raw
            // polygon exceeded the vertex cap.
            BrepFlattener.SimplifyWarning = msg =>
            {
                if (msg != null) RhinoApp.WriteLine(msg);
            };

            var polygons = new List<Polygon>();
            // Phase 7b: parallel per-part inner-loop table indexed by polygons[i].
            // Bypasses the nesting engine entirely; consumed only at draw time.
            var innerLoopsPerPart = new List<IReadOnlyList<Curve>>();
            for (int i = 0; i < breps.Count; i++)
            {
                var flat = BrepFlattener.Flatten(breps[i], doc);
                if (flat == null)
                {
                    RhinoApp.WriteLine($"Part {i + 1}: could not flatten — skipped.");
                }
                else
                {
                    polygons.Add(flat.Outer);
                    innerLoopsPerPart.Add(flat.InnerLoops);
                }
            }

            if (squishedCount > 0)
            {
                RhinoApp.WriteLine(
                    $"Note: {squishedCount} part(s) used Squish flattening — dimensions may be approximate for curved parts.");
            }

            if (polygons.Count == 0)
            {
                RhinoApp.WriteLine("No parts could be flattened. Aborting.");
                return Rhino.Commands.Result.Failure;
            }

            NestRequest request;
            try
            {
                request = new NestRequest(
                    polygons, sheetW, sheetH, sheetT, margin, spacing, rotStep,
                    algorithm,
                    allowMirror,
                    TimeSpan.FromSeconds(timeBudgetSeconds));
            }
            catch (ArgumentException ex)
            {
                RhinoApp.WriteLine($"Invalid parameters: {ex.Message}");
                return Rhino.Commands.Result.Failure;
            }

            var dialog = new ProgressDialog("Nesting... Please Wait");
            dialog.Show();
            Application.Instance.RunIteration();

            NestResponse response;
            try
            {
                var engine = new NestingEngine
                {
                    ProgressCallback = (frac, msg) =>
                    {
                        dialog.UpdateStatus(msg);
                        Application.Instance.RunIteration();
                    },
                    DiagnosticCallback = msg => RhinoApp.WriteLine(msg)
                };
                response = engine.Nest(request);
            }
            catch (Exception ex)
            {
                dialog.Close();
                RhinoApp.WriteLine($"Nesting failed: {ex.Message}");
                return Rhino.Commands.Result.Failure;
            }
            finally
            {
                if (dialog.Visible) dialog.Close();
            }

            // Diagnostic: print every overlapping (or near-overlapping) pair the verifier
            // sees, with bbox + intersection area. Helps narrow engine-vs-verifier
            // disagreement to a specific pair when verification fails.
            FinalVerifier.DiagnosticCallback = msg => RhinoApp.WriteLine(msg);
            var verification = FinalVerifier.Verify(response.Placements);
            if (!verification.IsValid)
            {
                string msg = $"{verification.OverlappingPartCount} parts overlap — report this bug";
                RhinoApp.WriteLine(msg);
                MessageBox.Show(msg, "SeaNest Nesting Error", MessageBoxButtons.OK, MessageBoxType.Error);
                return Rhino.Commands.Result.Failure;
            }

            if (response.Placements.Count == 0)
            {
                RhinoApp.WriteLine("No parts could be placed.");
                return Rhino.Commands.Result.Nothing;
            }

            DrawNestingResult(doc, response, sheetW, sheetH, sheetT, margin, inToModel, isMetric, innerLoopsPerPart);

            int mirrored = response.Placements.Count(p => p.IsMirrored);
            string mirrorNote = mirrored > 0 ? $", {mirrored} mirrored" : "";

            RhinoApp.WriteLine(
                $"SeaNest [{algorithm}]: placed {response.Placements.Count}/{polygons.Count} parts on {response.SheetCount} sheet(s){mirrorNote}, " +
                $"utilization {response.Utilization:P1}, {response.ElapsedTime.TotalSeconds:F2}s.");

            if (response.UnplacedIndices.Count > 0)
            {
                var originalNumbers = response.UnplacedIndices.Select(i => (i + 1).ToString());
                RhinoApp.WriteLine($"Unplaced parts: {string.Join(", ", originalNumbers)}");
            }

            return Rhino.Commands.Result.Success;
        }

        // ------------------------------------------------------------------
        // Drawing: extracted into a helper so SeaNestReNestCommand can reuse it
        // ------------------------------------------------------------------

        internal static void DrawNestingResult(
            RhinoDoc doc,
            NestResponse response,
            double sheetW, double sheetH, double sheetT,
            double margin, double inToModel, bool isMetric,
            IReadOnlyList<IReadOnlyList<Curve>> innerLoopsPerPart = null)
        {
            int layerNestedIdx = EnsureLayer(doc, LayerNested, System.Drawing.Color.Black);
            int layerSheetsIdx = EnsureLayer(doc, LayerSheets, System.Drawing.Color.Black);
            int layerLabelsIdx = EnsureLayer(doc, LayerLabels, System.Drawing.Color.Magenta);
            int dimStyleIdx = EnsureDimStyle(doc, DimStyleName);
            var dimStyle = doc.DimStyles[dimStyleIdx];

            double labelHeight = LabelHeightIn * inToModel;
            double labelStride = labelHeight * LabelStrideFactor;
            double sheetStride = sheetH + labelStride;

            var createdObjectIds = new List<Guid>();

            for (int s = 0; s < response.SheetCount; s++)
            {
                double yOffset = s * sheetStride;

                var sheetCurve = PolygonToCurve.SheetRectangle(sheetW, sheetH, yOffset);
                var sheetAttrs = new ObjectAttributes { LayerIndex = layerSheetsIdx };
                var sheetId = doc.Objects.AddCurve(sheetCurve, sheetAttrs);
                if (sheetId != Guid.Empty) createdObjectIds.Add(sheetId);

                string labelText = FormatSheetLabel(sheetW, sheetH, sheetT, isMetric);
                bool lengthIsX = sheetW >= sheetH;

                Point3d labelOrigin;
                Plane labelPlane;
                if (lengthIsX)
                {
                    labelOrigin = new Point3d(sheetW / 2.0, yOffset + sheetH + labelHeight * 0.5, 0);
                    labelPlane = new Plane(labelOrigin, Vector3d.ZAxis);
                }
                else
                {
                    labelOrigin = new Point3d(sheetW + labelHeight * 0.5, yOffset + sheetH / 2.0, 0);
                    labelPlane = new Plane(labelOrigin, Vector3d.YAxis, -Vector3d.XAxis);
                }

                var textEntity = TextEntity.Create(labelText, labelPlane, dimStyle, false, 0, 0);
                if (textEntity != null)
                {
                    textEntity.TextHeight = labelHeight;
                    textEntity.DimensionScale = 1.0;
                    textEntity.Justification = TextJustification.MiddleCenter;

                    var labelAttrs = new ObjectAttributes { LayerIndex = layerLabelsIdx };
                    var labelId = doc.Objects.AddText(textEntity, labelAttrs);
                    if (labelId != Guid.Empty) createdObjectIds.Add(labelId);
                }
            }

            foreach (var pp in response.Placements)
            {
                double yOffset = pp.Sheet * sheetStride;
                var partCurve = PolygonToCurve.ToCurve(pp.PlacedPolygon, yOffset);
                var partAttrs = new ObjectAttributes { LayerIndex = layerNestedIdx };
                var partId = doc.Objects.AddCurve(partCurve, partAttrs);
                if (partId != Guid.Empty) createdObjectIds.Add(partId);

                // Phase 7b: ride-along inner-loop cut curves (pipe holes, hatch
                // cutouts). Engine never sees these; we apply the same
                // mirror→rotate→translate that the placed outer received.
                if (innerLoopsPerPart != null && pp.OriginalIndex < innerLoopsPerPart.Count)
                {
                    var loops = innerLoopsPerPart[pp.OriginalIndex];
                    if (loops != null)
                    {
                        foreach (var loop in loops)
                        {
                            var transformed = PolygonToCurve.ToCurveFromOriginal(loop, pp, yOffset);
                            var loopId = doc.Objects.AddCurve(transformed, partAttrs);
                            if (loopId != Guid.Empty) createdObjectIds.Add(loopId);
                        }
                    }
                }
            }

            var rotationTransform = Transform.Rotation(Math.PI / 2.0, Vector3d.ZAxis, Point3d.Origin);
            foreach (var id in createdObjectIds)
            {
                doc.Objects.Transform(id, rotationTransform, true);
            }

            doc.Views.Redraw();
        }

        // ------------------------------------------------------------------
        // Prompt + layer + label helpers (internal so sibling commands can reuse)
        // ------------------------------------------------------------------

        internal static bool PromptForDouble(RhinoDoc doc, string prompt, double defaultValue, out double value)
        {
            var gn = new GetNumber();
            gn.SetCommandPrompt(prompt);
            gn.SetDefaultNumber(defaultValue);
            gn.SetLowerLimit(0.0, true);
            var res = gn.Get();
            if (res != GetResult.Number) { value = 0; return false; }
            value = gn.Number();
            return true;
        }

        internal static bool PromptForRotation(out RotationStep step)
        {
            var go = new GetOption();
            go.SetCommandPrompt("Rotation step");
            int idx90 = go.AddOption("Every90");
            int idx45 = go.AddOption("Every45");
            int idx15 = go.AddOption("Every15");
            int idx5 = go.AddOption("Every5Slow");
            go.AcceptNothing(true);
            var res = go.Get();

            if (res == GetResult.Nothing) { step = RotationStep.Every90; return true; }
            if (res != GetResult.Option) { step = RotationStep.Every90; return false; }

            int sel = go.Option().Index;
            if (sel == idx45) step = RotationStep.Every45;
            else if (sel == idx15) step = RotationStep.Every15;
            else if (sel == idx5) step = RotationStep.Every5Slow;
            else step = RotationStep.Every90;
            return true;
        }

        /// <summary>
        /// Algorithm picker. Defaults to BLF on Enter to preserve Phase 1 behavior.
        /// </summary>
        internal static bool PromptForAlgorithm(out NestingAlgorithm algorithm)
        {
            var go = new GetOption();
            go.SetCommandPrompt("Algorithm");
            int idxBlf = go.AddOption("BLF");
            int idxGreedy = go.AddOption("NFP_Greedy");
            int idxAnnealed = go.AddOption("NFP_Annealed");
            go.AcceptNothing(true);
            var res = go.Get();

            if (res == GetResult.Nothing) { algorithm = DefaultAlgorithm; return true; }
            if (res != GetResult.Option) { algorithm = DefaultAlgorithm; return false; }

            int sel = go.Option().Index;
            if (sel == idxGreedy) algorithm = NestingAlgorithm.NFP_Greedy;
            else if (sel == idxAnnealed) algorithm = NestingAlgorithm.NFP_Annealed;
            else algorithm = NestingAlgorithm.BLF;
            return true;
        }

        /// <summary>
        /// Yes/No option prompt. Defaults to <paramref name="defaultValue"/> on Enter.
        /// </summary>
        internal static bool PromptForBool(string prompt, bool defaultValue, out bool value)
        {
            var go = new GetOption();
            go.SetCommandPrompt(prompt);
            int idxYes = go.AddOption("Yes");
            int idxNo = go.AddOption("No");
            go.AcceptNothing(true);
            var res = go.Get();

            if (res == GetResult.Nothing) { value = defaultValue; return true; }
            if (res != GetResult.Option) { value = defaultValue; return false; }

            value = go.Option().Index == idxYes;
            return true;
        }

        private static int EnsureLayer(RhinoDoc doc, string name, System.Drawing.Color color)
        {
            int idx = doc.Layers.FindByFullPath(name, -1);
            if (idx >= 0) return idx;
            var layer = new Layer { Name = name, Color = color };
            return doc.Layers.Add(layer);
        }

        private static int EnsureDimStyle(RhinoDoc doc, string name)
        {
            var existing = doc.DimStyles.FindName(name);
            if (existing != null)
            {
                existing.DimensionScale = 1.0;
                doc.DimStyles.Modify(existing, existing.Id, true);
                return existing.Index;
            }

            var ds = new Rhino.DocObjects.DimensionStyle
            {
                Name = name,
                DimensionScale = 1.0,
                TextHeight = 1.0
            };
            return doc.DimStyles.Add(ds, false);
        }

        private static string FormatSheetLabel(double w, double h, double t, bool isMetric)
        {
            string unit = isMetric ? "mm" : "\"";
            return $"{FormatNumber(w)}{unit} x {FormatNumber(h)}{unit} x {FormatNumber(t)}{unit}";
        }

        private static string FormatNumber(double v)
        {
            if (Math.Abs(v - Math.Round(v)) < 1e-9)
                return ((int)Math.Round(v)).ToString();
            return v.ToString("0.###");
        }
    }
}