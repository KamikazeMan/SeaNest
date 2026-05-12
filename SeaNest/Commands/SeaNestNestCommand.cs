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

        private const string DimStyleName = "SeaNest_Labels";

        private const double DefaultSheetWidthIn = 96.0;
        private const double DefaultSheetHeightIn = 48.0;
        private const double DefaultThicknessIn = 0.25;
        private const double DefaultMarginIn = 0.25;
        private const double DefaultSpacingIn = 0.25;
        private const double LabelHeightIn = 6.0;
        private const double LabelStrideFactor = 3.0;

        // Phase 8 — per-part labels (object name) emitted on the SeaNest_Labels layer.
        // Smaller than the sheet-size labels; dimension scale 5 makes them visible at
        // boat-model zoom levels while keeping underlying geometry at 0.5".
        private const double PartLabelHeightIn = 0.5;
        private const double PartLabelScale = 5.0;
        private const string PartLabelFontFace = "SLF-RHN Architect";

        // Phase 10 — pole-of-inaccessibility precision for label anchor placement.
        // 0.5" is well below the 0.5"-tall label glyph (2.5" after dim-scale 5),
        // so sub-half-inch label shifts within the part body are visually
        // imperceptible. Tuned for plate parts in the 10"-300" range.
        private const double PartLabelPolelabelPrecisionIn = 0.5;

        // Phase 11 — aspect-ratio threshold above which the label rotation
        // overrides pp.RotationDeg with the placed polygon's PCA principal
        // axis. 5:1 cleanly separates "visually elongated strip" (where the
        // long axis is the natural reading direction) from "roughly square
        // or chunky" parts (where PCA is unstable and Phase 8's part-rotation
        // alignment is more intuitive).
        private const double PartLabelAspectThresholdForRotate = 5.0;

        // Phase 13 — auto-align elongated parts so their principal axis points
        // along world +X before the engine sees them. This makes Every90 /
        // Every45 rotation steps find fits for parts that were modeled at
        // non-axis-aligned angles in Rhino (e.g. boat-side panels in boat
        // coordinates, 15-25° from world XY). Engine receives pre-aligned
        // polygons + matching pre-rotated inner-loop and outer-curve data;
        // it has no awareness of the alignment.
        //
        // Aspect-ratio threshold: parts more elongated than 2:1 benefit from
        // alignment. Below this, the PCA principal axis is geometrically
        // unstable (Phase 11 audit) and the part's bbox is similar in both
        // dimensions, so any rotation-step already finds a fitting orientation
        // without help — pre-rotation would just risk arbitrary axis flipping.
        private const double AutoAlignAspectThreshold = 2.0;

        // Skip pre-rotation when the alignment angle is below this — avoids
        // floating-point micro-rotations on already-aligned parts that would
        // churn vertex data without changing fit. ~0.5° in radians.
        private const double AutoAlignAngleEpsilon = 0.0087;

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
            // Phase 8: parallel-per-brep name capture from each source object's
            // attributes. Null when the object has no Name set — we'll skip those
            // at label-emit time. The list stays aligned with `breps` so the same
            // skip-on-no-Brep semantics propagate downstream.
            var brepNames = new List<string>();
            // Phase 19b: track plate ObjectIds so the member-selection prompt
            // can filter out any plate the user accidentally re-picks as a
            // scribe source (defensive against self-intersection garbage).
            var plateObjectIds = new HashSet<Guid>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var obj = go.Object(i);
                var brep = obj.Brep();
                if (brep == null)
                {
                    var ext = obj.Geometry() as Extrusion;
                    if (ext != null) brep = ext.ToBrep();
                }
                if (brep != null)
                {
                    breps.Add(brep);
                    brepNames.Add(obj.Object()?.Attributes?.Name);
                    plateObjectIds.Add(obj.ObjectId);
                }
            }

            if (breps.Count == 0)
            {
                RhinoApp.WriteLine("No valid Breps selected.");
                return Rhino.Commands.Result.Nothing;
            }

            // Phase 19b — structural-member selection for scribe lines. Optional:
            // pressing Enter skips the prompt with no scribe sources, behavior
            // identical to pre-19b. Members can be Breps, Extrusions, or
            // single-face Surfaces (frames, longitudinals, bulkheads, brackets).
            // EnablePreSelect(false) so any leftover pre-selection from the plate
            // prompt isn't re-consumed here. Filter by ObjectId against
            // plateObjectIds so a user who fat-fingers a plate into the member
            // selection doesn't get self-intersection garbage.
            //
            // Phase 19b.0.1: result handling uses CommandResult() + ObjectCount,
            // matching the plate prompt's pattern (see line 130 block). The prior
            // GetResult.Object-vs-Nothing check fired silently empty when
            // AcceptNothing(true) caused the post-pick Enter to return
            // GetResult.Nothing in some Rhino 8 configurations — picks were
            // accessible via ObjectCount but the if/else discarded them.
            // CommandResult only distinguishes Success / Cancel, so picks
            // survive whatever GetResult value the API actually returned.
            var memberBreps = new List<Brep>();
            var goMembers = new GetObject();
            goMembers.SetCommandPrompt("Select structural members for scribe lines, or press Enter to skip");
            goMembers.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion | ObjectType.Surface;
            goMembers.GroupSelect = true;
            goMembers.SubObjectSelect = false;
            goMembers.AcceptNothing(true);
            goMembers.EnablePreSelect(false, true);
            goMembers.GetMultiple(0, 0);
            var memberCmdResult = goMembers.CommandResult();
            if (memberCmdResult == Rhino.Commands.Result.Cancel)
                return memberCmdResult;
            // Success path covers both "picked + Enter" and "Enter immediately"
            // — distinguished only by goMembers.ObjectCount.

            int selfPickedCount = 0;
            for (int i = 0; i < goMembers.ObjectCount; i++)
            {
                var obj = goMembers.Object(i);
                if (plateObjectIds.Contains(obj.ObjectId))
                {
                    selfPickedCount++;
                    continue;
                }
                var mbrep = obj.Brep();
                if (mbrep == null)
                {
                    var ext = obj.Geometry() as Extrusion;
                    if (ext != null) mbrep = ext.ToBrep();
                }
                if (mbrep == null)
                {
                    var srf = obj.Geometry() as Surface;
                    if (srf != null) mbrep = Brep.CreateFromSurface(srf);
                }
                if (mbrep != null) memberBreps.Add(mbrep);
            }
            if (selfPickedCount > 0)
            {
                RhinoApp.WriteLine(
                    $"{selfPickedCount} plate(s) re-picked as scribe members — filtered out " +
                    "(plates can't scribe themselves).");
            }

            // Phase 19b.0.1 temporary diagnostic — verifies the fix populates
            // memberBreps. Tagged [scribe-diag] for easy strip in Phase 19b.0.2.
            RhinoApp.WriteLine(
                $"[scribe-diag] memberBreps populated with {memberBreps.Count} member(s) after selection.");

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

            // Phase 19b: surface scribe warnings (Step 3 Unroll / Step 4 Squish
            // dropped scribes) to the user and tally how many parts dropped
            // scribes — single end-of-command summary line below.
            int scribeDroppedCount = 0;
            BrepFlattener.ScribeWarning = msg =>
            {
                if (msg == null) return;
                RhinoApp.WriteLine(msg);
                scribeDroppedCount++;
            };

            var polygons = new List<Polygon>();
            // Phase 7b: parallel per-part inner-loop table indexed by polygons[i].
            // Bypasses the nesting engine entirely; consumed only at draw time.
            var innerLoopsPerPart = new List<IReadOnlyList<Curve>>();
            // Phase 9a: parallel per-part native outer curve, same alignment
            // convention. Used at draw time for high-fidelity outer rendering;
            // null entries fall back to drawing the placed polygon.
            var outerCurvePerPart = new List<Curve>();
            // Phase 8: parallel per-part Name table, same alignment convention.
            var namesPerPart = new List<string>();
            // Phase 19b: parallel per-part scribe-line table. Emitted on the
            // SeaNest_Scribe layer at draw time; transformed alongside the
            // outer via the same PlacementResult.Transform.
            var scribeLinesPerPart = new List<IReadOnlyList<Curve>>();
            for (int i = 0; i < breps.Count; i++)
            {
                var flat = BrepFlattener.Flatten(breps[i], doc, memberBreps);
                if (flat == null)
                {
                    RhinoApp.WriteLine(
                        $"Part {i + 1}: could not flatten (no twin pair, no planar face, " +
                        "unroll failed, squish failed) — skipped. " +
                        "Check that this part is a closed solid or single planar surface.");
                }
                else
                {
                    var polygon = flat.OuterPolygon;
                    var innerLoops = flat.InnerLoops;
                    var outerCurve = flat.OuterCurve;
                    // Phase 19b: scribe lines ride along with the outer through
                    // every transform the engine applies, just like inner loops.
                    var scribeLines = flat.ScribeLines;

                    // Phase 13: auto-align elongated parts so the principal axis
                    // points along world +X before the engine ingests them. All
                    // four of (polygon, inner loops, outer curve, scribe lines)
                    // are rotated by the same angle around the same pivot (the
                    // polygon's centroid, captured once before rotation since
                    // it's rotation-invariant about itself) so they stay in
                    // lockstep. Skip for near-square parts (PCA unstable) and
                    // for already-aligned parts (avoids float churn).
                    if (polygon.AspectRatio > AutoAlignAspectThreshold)
                    {
                        double alignAngle = -polygon.PrincipalAxisAngle;
                        if (Math.Abs(alignAngle) > AutoAlignAngleEpsilon)
                        {
                            var pivot = polygon.Centroid;

                            polygon = polygon.RotateAround(pivot, alignAngle);

                            var pivot3d = new Point3d(pivot.X, pivot.Y, 0);
                            var rotationXform = Transform.Rotation(alignAngle, Vector3d.ZAxis, pivot3d);

                            if (innerLoops != null && innerLoops.Count > 0)
                            {
                                var rotatedLoops = new List<Curve>(innerLoops.Count);
                                foreach (var loop in innerLoops)
                                {
                                    var dup = loop.DuplicateCurve();
                                    if (dup != null && dup.Transform(rotationXform))
                                        rotatedLoops.Add(dup);
                                    // else: drop this loop only; rest of the
                                    // part continues. Polygon and outer-curve
                                    // are still consistent with the parts that
                                    // did rotate; a single missing inner-loop
                                    // is a graceful local degradation.
                                }
                                innerLoops = rotatedLoops;
                            }

                            if (outerCurve != null)
                            {
                                var dup = outerCurve.DuplicateCurve();
                                if (dup != null && dup.Transform(rotationXform))
                                    outerCurve = dup;
                                else
                                    outerCurve = null;   // fall back to Phase 9a's
                                                          // polygon-rendered outer
                                                          // at draw time.
                            }

                            if (scribeLines != null && scribeLines.Count > 0)
                            {
                                var rotatedScribes = new List<Curve>(scribeLines.Count);
                                foreach (var s in scribeLines)
                                {
                                    var dup = s.DuplicateCurve();
                                    if (dup != null && dup.Transform(rotationXform))
                                        rotatedScribes.Add(dup);
                                    // else: drop this scribe; rest of the part
                                    // continues. Same graceful-local-degradation
                                    // policy as inner loops.
                                }
                                scribeLines = rotatedScribes;
                            }
                        }
                    }

                    polygons.Add(polygon);
                    innerLoopsPerPart.Add(innerLoops);
                    outerCurvePerPart.Add(outerCurve);
                    namesPerPart.Add(brepNames[i]);
                    scribeLinesPerPart.Add(scribeLines);
                }
            }

            if (squishedCount > 0)
            {
                RhinoApp.WriteLine(
                    $"Note: {squishedCount} part(s) used Squish flattening — dimensions may be approximate for curved parts.");
            }

            if (scribeDroppedCount > 0)
            {
                RhinoApp.WriteLine(
                    $"Note: scribes dropped for {scribeDroppedCount} part(s) on Unroll/Squish paths (see warnings above).");
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

            DrawNestingResult(
                doc, response, sheetW, sheetH, sheetT, margin, inToModel, isMetric,
                innerLoopsPerPart, namesPerPart, outerCurvePerPart,
                scribeLinesPerPart: scribeLinesPerPart);

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
            IReadOnlyList<IReadOnlyList<Curve>> innerLoopsPerPart = null,
            IReadOnlyList<string> namesPerPart = null,
            IReadOnlyList<Curve> outerCurvePerPart = null,
            IReadOnlyList<IReadOnlyList<Curve>> scribeLinesPerPart = null)
        {
            // Phase 8: SLF-RHN Architect font lookup, plus a one-per-command
            // warning if the font isn't available (Rhino bundles it, but a
            // misconfigured install could miss it). Defensive: fall through
            // to dim-style default so labels still emit.
            var partLabelFont = Rhino.DocObjects.Font.FromQuartetProperties(PartLabelFontFace, false, false);
            bool partLabelFontMissingWarned = false;

            int layerNestedIdx = EnsureLayer(doc, SeaNestLayers.Nested, System.Drawing.Color.Black);
            int layerSheetsIdx = EnsureLayer(doc, SeaNestLayers.Sheets, System.Drawing.Color.Black);
            int layerLabelsIdx = EnsureLayer(doc, SeaNestLayers.Labels, System.Drawing.Color.Magenta);
            int dimStyleIdx = EnsureDimStyle(doc, DimStyleName);
            var dimStyle = doc.DimStyles[dimStyleIdx];

            // Phase 19b — SeaNest_Scribe layer created lazily on first scribe
            // emission, so runs without scribe sources don't pollute the layer
            // panel with an empty SeaNest_Scribe layer. Cyan to stay distinct
            // from black-cut and magenta-label in Rhino viewport; the cutter
            // operator filters by layer name not color when routing toolpaths.
            int layerScribeIdx = -1;
            ObjectAttributes scribeAttrs = null;

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

                // Phase 9a — outer-draw branch. Two paths, both correctly placed:
                //
                //   Native-curve path: when BrepFlattener captured the outer's
                //   source Rhino curve (typical for Step 1/2/3 — flat or unrolled
                //   Breps), route it through ToCurveFromOriginal so the same
                //   placement transform the engine applied to the polygon
                //   (rotation, translation, mirror via source bbox-center X) lands
                //   the native curve in the right sheet position. Native subclass
                //   survives the rigid transform: NurbsCurve stays NurbsCurve,
                //   ArcCurve stays ArcCurve. Smooth output, faceting gone, DXF
                //   exports native SPLINE/ARC/CIRCLE entities for the outline.
                //
                //   Polygon-fallback path: when no native curve is available
                //   (Squish-path parts had no source curve, or an extraction
                //   failure produced null), draw the engine's PlacedPolygon
                //   directly — same behavior as pre-9a. Polygonal output, but
                //   correctly placed and verified by FinalVerifier.
                //
                // Both paths produce geometry on the same SeaNest_Nested layer
                // and both feed into the same per-sheet stacking and global
                // 90°-Z rotation, so downstream code sees no difference.
                Curve partCurve;
                if (outerCurvePerPart != null
                    && pp.OriginalIndex < outerCurvePerPart.Count
                    && outerCurvePerPart[pp.OriginalIndex] != null)
                {
                    partCurve = PolygonToCurve.ToCurveFromOriginal(
                        outerCurvePerPart[pp.OriginalIndex], pp, yOffset);
                }
                else
                {
                    partCurve = PolygonToCurve.ToCurve(pp.PlacedPolygon, yOffset);
                }

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

                // Phase 19b: ride-along scribe lines (structural reference
                // lines where frames / longitudinals / bulkheads intersect the
                // plate). Same transform as inner loops, separate layer so
                // the cutter operator can route them to a low-power-marking
                // or etch toolpath. Layer created lazily on first emit.
                if (scribeLinesPerPart != null && pp.OriginalIndex < scribeLinesPerPart.Count)
                {
                    var scribes = scribeLinesPerPart[pp.OriginalIndex];
                    if (scribes != null && scribes.Count > 0)
                    {
                        if (layerScribeIdx < 0)
                        {
                            layerScribeIdx = EnsureLayer(doc, SeaNestLayers.Scribe, System.Drawing.Color.Cyan);
                            scribeAttrs = new ObjectAttributes { LayerIndex = layerScribeIdx };
                        }
                        foreach (var scribe in scribes)
                        {
                            var transformed = PolygonToCurve.ToCurveFromOriginal(scribe, pp, yOffset);
                            var scribeId = doc.Objects.AddCurve(transformed, scribeAttrs);
                            if (scribeId != Guid.Empty) createdObjectIds.Add(scribeId);
                        }
                    }
                }

                // Phase 8: object-name label at the placed-polygon centroid,
                // rotated to match the part's RotationDeg. Position uses
                // pp.PlacedPolygon.Centroid directly — the engine already
                // applied mirror/rotate/translate, so no transform math here
                // and no risk of mirror-axis bugs. Rotation is pure
                // (no mirror flip on the glyph) so text stays readable on
                // mirrored placements. Skipped when the source object has
                // no Name attribute.
                if (namesPerPart != null && pp.OriginalIndex < namesPerPart.Count)
                {
                    string name = namesPerPart[pp.OriginalIndex];
                    if (!string.IsNullOrWhiteSpace(name))
                    {
                        // Phase 10: pole-of-inaccessibility instead of centroid so
                        // labels land inside the part body even when the polygon is
                        // concave (notched plates, fantail panels with deep inward
                        // curves where the centroid can fall outside the solid).
                        var c = pp.PlacedPolygon.PoleOfInaccessibility(
                            PartLabelPolelabelPrecisionIn * inToModel);
                        var labelOrigin = new Point3d(c.X, c.Y + yOffset, 0);
                        var labelPartPlane = new Plane(labelOrigin, Vector3d.ZAxis);
                        // Phase 11: elongated parts read along their long axis,
                        // not along the engine's placement rotation. PCA's
                        // principal-axis angle is the visual long axis; we use
                        // it only when AspectRatio crosses the threshold, so
                        // near-square parts (where PCA is unstable) keep
                        // Phase 8's rotate-with-the-part behavior.
                        double rotRad = pp.PlacedPolygon.AspectRatio > PartLabelAspectThresholdForRotate
                            ? pp.PlacedPolygon.PrincipalAxisAngle
                            : pp.RotationDeg * Math.PI / 180.0;
                        labelPartPlane.Rotate(rotRad, Vector3d.ZAxis);

                        var partLabel = TextEntity.Create(name, labelPartPlane, dimStyle, false, 0, 0);
                        if (partLabel != null)
                        {
                            partLabel.TextHeight = PartLabelHeightIn * inToModel;
                            partLabel.DimensionScale = PartLabelScale;
                            partLabel.Justification = TextJustification.MiddleCenter;
                            if (partLabelFont != null)
                            {
                                partLabel.Font = partLabelFont;
                            }
                            else if (!partLabelFontMissingWarned)
                            {
                                RhinoApp.WriteLine(
                                    $"Warning: '{PartLabelFontFace}' font not available; " +
                                    "part labels will use the default font.");
                                partLabelFontMissingWarned = true;
                            }

                            var partLabelAttrs = new ObjectAttributes { LayerIndex = layerLabelsIdx };
                            var partLabelId = doc.Objects.AddText(partLabel, partLabelAttrs);
                            if (partLabelId != Guid.Empty) createdObjectIds.Add(partLabelId);
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