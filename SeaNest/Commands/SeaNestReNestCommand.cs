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
    /// <summary>
    /// Re-nest already-flat 2D curves (e.g. output from a previous SeaNestNest run, or
    /// curves from any other source). Skips Brep flattening entirely — the selected curves
    /// ARE the polygons.
    ///
    /// Useful for iterating on sheet size / margin / spacing / rotation parameters
    /// without re-cooking the Brep geometry (which is slow for curved parts that go through Squish).
    /// </summary>
    public class SeaNestReNestCommand : RhinoCommand
    {
        public override string EnglishName => "SeaNestReNest";

        private const double DefaultSheetWidthIn = 96.0;
        private const double DefaultSheetHeightIn = 48.0;
        private const double DefaultThicknessIn = 0.25;
        private const double DefaultMarginIn = 0.25;
        private const double DefaultSpacingIn = 0.25;

        // Phase 18 — match BrepFlattener's vertex cap so the engine never sees
        // a curve's raw chord-tessellation. Original Rhino Curve is preserved
        // separately for visual output (see outerCurvePerPart below).
        private const int MaxVertices = 500;
        private const double EscalationFactor = 2.0;
        private const int MaxEscalations = 3;

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

            if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet width", DefaultSheetWidthIn * inToModel, out sheetW)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet height", DefaultSheetHeightIn * inToModel, out sheetH)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Sheet thickness", DefaultThicknessIn * inToModel, out sheetT)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Margin", DefaultMarginIn * inToModel, out margin)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForDouble(doc, "Spacing", DefaultSpacingIn * inToModel, out spacing)) return Rhino.Commands.Result.Cancel;
            if (!SeaNestNestCommand.PromptForRotation(out rotStep)) return Rhino.Commands.Result.Cancel;

            var go = new GetObject();
            go.SetCommandPrompt("Select closed 2D curves to re-nest");
            go.GeometryFilter = ObjectType.Curve;
            go.GeometryAttributeFilter = GeometryAttributeFilter.ClosedCurve;
            go.GroupSelect = true;
            go.SubObjectSelect = false;
            go.GetMultiple(1, 0);
            if (go.CommandResult() != Rhino.Commands.Result.Success)
                return go.CommandResult();

            double modelTol = doc.ModelAbsoluteTolerance;
            double discretizeTol = modelTol * 0.1;
            double angleTolRad = RhinoMath.ToRadians(0.5);

            // Phase 18 — parallel lists. Engine sees the simplified polygon;
            // DrawNestingResult sees the original Rhino Curve. This is the
            // same architecture SeaNestNest established for outer curves in
            // Phase 9a (outerCurvePerPart). Native NURBS/arc/polyline
            // representation survives the round-trip through the engine.
            //
            // TODO (Phase 18.1): inner-loop association. Today each closed
            // curve becomes its own part; outer + hole curves selected
            // together get nested as independent parts. A future phase
            // should detect Curve.Contains topology to pair holes with
            // their outer and plumb them through innerLoopsPerPart.
            var polygons = new List<Polygon>();
            var outerCurvePerPart = new List<Curve>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var obj = go.Object(i);
                var curve = obj.Curve();
                if (curve == null)
                {
                    RhinoApp.WriteLine($"Curve {i + 1}: not a curve — skipped.");
                    continue;
                }

                var converted = CurveToPolygon(curve, discretizeTol, angleTolRad, MaxVertices);
                if (converted == null)
                {
                    RhinoApp.WriteLine($"Curve {i + 1}: could not convert — skipped.");
                    continue;
                }

                var (poly, originalCurve, finalTol) = converted.Value;
                if (finalTol > discretizeTol + 1e-12)
                {
                    RhinoApp.WriteLine(
                        $"Curve {i + 1}: simplified at tol {finalTol:G3} to fit {MaxVertices}-vertex cap " +
                        $"(escalated from {discretizeTol:G3}). Visual fidelity preserved via native curve.");
                }
                polygons.Add(poly);
                outerCurvePerPart.Add(originalCurve);
            }

            if (polygons.Count == 0)
            {
                RhinoApp.WriteLine("No curves could be converted. Aborting.");
                return Rhino.Commands.Result.Failure;
            }

            NestRequest request;
            try
            {
                request = new NestRequest(polygons, sheetW, sheetH, sheetT, margin, spacing, rotStep);
            }
            catch (ArgumentException ex)
            {
                RhinoApp.WriteLine($"Invalid parameters: {ex.Message}");
                return Rhino.Commands.Result.Failure;
            }

            var dialog = new ProgressDialog("Re-Nesting... Please Wait");
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
                    }
                };
                response = engine.Nest(request);
            }
            catch (Exception ex)
            {
                dialog.Close();
                RhinoApp.WriteLine($"Re-nesting failed: {ex.Message}");
                return Rhino.Commands.Result.Failure;
            }
            finally
            {
                if (dialog.Visible) dialog.Close();
            }

            var verification = FinalVerifier.Verify(response.Placements);
            if (!verification.IsValid)
            {
                string msg = $"{verification.OverlappingPartCount} parts overlap — report this bug";
                RhinoApp.WriteLine(msg);
                MessageBox.Show(msg, "SeaNest Re-Nest Error", MessageBoxButtons.OK, MessageBoxType.Error);
                return Rhino.Commands.Result.Failure;
            }

            if (response.Placements.Count == 0)
            {
                RhinoApp.WriteLine("No curves could be placed.");
                return Rhino.Commands.Result.Nothing;
            }

            // Phase 18 — explicit nulls for innerLoopsPerPart and namesPerPart
            // document that those features are deferred (see Phase 18.1
            // TODO above for inner-loop association).
            SeaNestNestCommand.DrawNestingResult(
                doc, response, sheetW, sheetH, sheetT, margin, inToModel, isMetric,
                innerLoopsPerPart: null,
                namesPerPart: null,
                outerCurvePerPart: outerCurvePerPart);

            RhinoApp.WriteLine(
                $"SeaNest Re-Nest: placed {response.Placements.Count}/{polygons.Count} curves on {response.SheetCount} sheet(s), " +
                $"utilization {response.Utilization:P1}, {response.ElapsedTime.TotalSeconds:F2}s.");

            if (response.UnplacedIndices.Count > 0)
            {
                var originalNumbers = response.UnplacedIndices.Select(i => (i + 1).ToString());
                RhinoApp.WriteLine($"Unplaced curves: {string.Join(", ", originalNumbers)}");
            }

            return Rhino.Commands.Result.Success;
        }

        /// <summary>
        /// Convert a closed 2D curve to a <see cref="Polygon"/> on its own best-fit plane.
        /// For a curve that already lies on world XY (typical re-nest input), this resolves
        /// to XY directly. For tilted/rotated 2D curves it still flattens cleanly.
        ///
        /// Phase 18: returns the original Rhino curve alongside the polygon so the
        /// draw path can preserve native NURBS/arc/polyline representation
        /// (mirrors SeaNestNest's outerCurvePerPart). Applies Polygon.SimplifyToTarget
        /// with the same vertex-cap escalation pattern BrepFlattener uses.
        /// </summary>
        private static (Polygon polygon, Curve originalCurve, double finalTolerance)? CurveToPolygon(
            Curve curve, double discretizeTol, double angleTolRad, int maxVertices)
        {
            if (!curve.IsClosed)
                return null;

            // Find the plane the curve lies on (should be planar for a nesting-ready 2D curve).
            if (!curve.TryGetPlane(out Plane plane, discretizeTol * 10.0))
            {
                // Fall back to world XY if the curve is essentially 2D but not perfectly planar.
                plane = Plane.WorldXY;
            }

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

            // Phase 18 — cap engine-input vertex count using the same DP +
            // escalation pattern BrepFlattener uses for Brep-source polygons.
            // The native Rhino curve is returned separately so the visual
            // output is unaffected by this simplification.
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
                // Defensive: SimplifyToTarget rejects bad inputs. Fall back to
                // the unsimplified polygon so the part still gets a chance to nest.
                simplified = poly;
                finalTol = discretizeTol;
            }

            return (simplified, curve, finalTol);
        }
    }
}