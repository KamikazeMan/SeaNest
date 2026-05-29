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
using SeaNest.Nesting.Core.Overlap;
using SeaNest.RhinoAdapters;
#if NET7_0
using Google.OrTools.Sat;
#endif

namespace SeaNest.Commands
{
    // Phase 29 CP-SAT-on-frames spike (THROWAWAY).
    //
    // Gating question: can a TWO-frame NFP-disjunction CP-SAT model solve in
    // tolerable time, how many disjunction booleans does it need, and does the
    // solved placement actually interlock the frames? Select two large hull
    // frames: A is fixed at origin, B is placed by the solver.
    //
    // Encoding: B's reference point (Bx,By) must lie OUTSIDE the no-fit polygon
    // of B relative to A (forbidden region). Per the spike spec, "outside" is
    // encoded as a disjunction over the NFP outer-contour edges: a boolean per
    // edge, reified to the half-plane "B is on the outer (right) side of that
    // edge", with AddBoolOr requiring at least one to hold.
    //
    // CORRECTNESS CAVEAT: "outside >= 1 edge" is exactly equivalent to "outside
    // the polygon" only for CONVEX polygons. The NFP here is CONCAVE (166 verts
    // per Phase 29 NFP validation), so this all-edges disjunction is an
    // OVER-PERMISSIVE RELAXATION: it can admit points inside reflex pockets,
    // i.e. a "feasible" CP-SAT solution may actually geometrically overlap. The
    // mathematically correct encoding decomposes the concave NFP into convex
    // pieces and requires "outside every piece". For this spike we implement
    // the spec'd relaxation BUT add a ground-truth OverlapChecker test at the
    // solution so the report states whether the solved placement is genuinely
    // overlap-free or a false positive. (Holes: the validation found 0 holes;
    // we guard and report any, but the spike assumes none. A real hole would
    // be an additional forbidden region requiring its own "inside-all-edges"
    // conjunction subtracted from the feasible set.)
    public class SeaNestCpSatNfp2 : Command
    {
        public override string EnglishName => "SeaNestCpSatNfp2";

        // Real job spacing (part-to-part clearance), not 0.0 — we want a
        // placement with manufacturing clearance.
        private const double Spacing = 0.25;

        // Grid scale: 1 integer unit = 0.05". Fine relative to the 0.25"
        // spacing, coarse enough to bound the integer domains.
        private const double GridUnitInches = 0.05;
        private const double Scale = 1.0 / GridUnitInches; // 20 units per inch

        private const double SheetWidthInches = 240.0;
        private const double SheetHeightInches = 72.0;

        private const double TimeLimitSeconds = 60.0;

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
#if NET7_0
            var lines = new List<string>
            {
                $"Phase 29 CP-SAT NFP 2-frame spike run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                $"Spacing: {Spacing:F3}\"  |  Grid: 1 unit = {GridUnitInches:F3}\" ({Scale:F0} units/inch)  |  " +
                $"Sheet: {SheetWidthInches:F0}x{SheetHeightInches:F0}\"  |  TimeLimit: {TimeLimitSeconds:F0}s",
                ""
            };

            try
            {
                // --- Select two frames: A (fixed) first, then B (moving). ---
                var go = new GetObject();
                go.SetCommandPrompt("Select TWO hull frames: A (fixed) first, then B (solver-placed)");
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

                // --- Same conversion path as SeaNestNfpValidate. ---
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

                // --- Real NFP of B relative to A (forbidden translations of B). ---
                IReadOnlyList<Polygon> nfp = NoFitPolygon.Compute(polyA, polyB, Spacing);
                if (nfp.Count == 0)
                {
                    lines.Add("NFP empty — frames do not interact; nothing to coordinate.");
                    FlushAndEcho(lines);
                    return Result.Success;
                }

                // Outer contour = largest CCW polygon; count any holes (CW).
                Polygon outer = null;
                int holeCount = 0;
                foreach (var p in nfp)
                {
                    if (p.IsCounterClockwise)
                    {
                        if (outer == null || p.AbsoluteArea > outer.AbsoluteArea) outer = p;
                    }
                    else holeCount++;
                }
                if (outer == null)
                {
                    lines.Add("No CCW outer contour in NFP — cannot build half-plane disjunction.");
                    FlushAndEcho(lines);
                    return Result.Failure;
                }
                if (holeCount > 0)
                    lines.Add($"WARNING: NFP has {holeCount} hole(s); spike ignores them (assumes none).");

                var outerPts = outer.Points;
                int edgeCount = outerPts.Count; // closed contour -> one edge per vertex
                lines.Add($"NFP outer contour: {edgeCount} edges => {edgeCount} disjunction booleans.");

                // --- Build CP-SAT model. ---
                var model = new CpModel();

                // B reference-point domain so B stays on-sheet. B's drawn polygon
                // is polyB.Translate(Bx,By); keep its bbox within [0,sheet].
                var bbB = polyB.BoundingBox;
                long bxLo = (long)Math.Ceiling((0.0 - bbB.MinX) * Scale);
                long bxHi = (long)Math.Floor((SheetWidthInches - bbB.MaxX) * Scale);
                long byLo = (long)Math.Ceiling((0.0 - bbB.MinY) * Scale);
                long byHi = (long)Math.Floor((SheetHeightInches - bbB.MaxY) * Scale);
                if (bxLo > bxHi || byLo > byHi)
                {
                    lines.Add($"B does not fit on sheet: Bx[{bxLo},{bxHi}] By[{byLo},{byHi}] (grid).");
                    FlushAndEcho(lines);
                    return Result.Failure;
                }

                IntVar Bx = model.NewIntVar(bxLo, bxHi, "Bx");
                IntVar By = model.NewIntVar(byLo, byHi, "By");

                // No-overlap disjunction over outer-contour edges. NFP is CCW, so
                // interior (forbidden) is LEFT of each directed edge; B is OUTSIDE
                // via edge e iff strictly RIGHT: cross(edge, P-Vi) <= -1 (grid).
                //   cross = (-dy)*Bx + (dx)*By + (dy*xi - dx*yi)
                var edgeBools = new List<ILiteral>(edgeCount);
                for (int i = 0; i < edgeCount; i++)
                {
                    var vi = outerPts[i];
                    var vj = outerPts[(i + 1) % edgeCount];

                    long xi = (long)Math.Round(vi.X * Scale);
                    long yi = (long)Math.Round(vi.Y * Scale);
                    long xj = (long)Math.Round(vj.X * Scale);
                    long yj = (long)Math.Round(vj.Y * Scale);

                    long dx = xj - xi;
                    long dy = yj - yi;
                    if (dx == 0 && dy == 0) continue; // degenerate edge

                    long c = dy * xi - dx * yi;          // constant term
                    long rhs = -1 - c;                    // cross <= -1

                    BoolVar be = model.NewBoolVar($"out_{i}");
                    // (-dy)*Bx + (dx)*By <= rhs   when be is true
                    var lhs = LinearExpr.WeightedSum(
                        new LinearExpr[] { Bx, By }, new long[] { -dy, dx });
                    model.Add(lhs <= rhs).OnlyEnforceIf(be);
                    edgeBools.Add(be);
                }
                model.AddBoolOr(edgeBools.ToArray()); // outside at least one edge

                // Objective: minimize combined bounding-box width in X (tight nest).
                long aMinX = (long)Math.Round(polyA.BoundingBox.MinX * Scale);
                long aMaxX = (long)Math.Round(polyA.BoundingBox.MaxX * Scale);
                long bMinXoff = (long)Math.Round(bbB.MinX * Scale);
                long bMaxXoff = (long)Math.Round(bbB.MaxX * Scale);

                long wide = (long)Math.Round((SheetWidthInches * 4) * Scale);
                IntVar maxX = model.NewIntVar(-wide, wide, "maxX");
                IntVar minX = model.NewIntVar(-wide, wide, "minX");
                model.Add(maxX >= aMaxX);
                model.Add(maxX >= Bx + bMaxXoff);
                model.Add(minX <= aMinX);
                model.Add(minX <= Bx + bMinXoff);
                model.Minimize(maxX - minX);

                // --- Solve deterministically. ---
                var solver = new CpSolver();
                solver.StringParameters =
                    $"max_time_in_seconds:{TimeLimitSeconds},random_seed:1,num_search_workers:1";

                CpSolverStatus status = solver.Solve(model);
                double wall = solver.WallTime();

                // Model size (proto). Guarded in case the accessor name differs.
                try
                {
                    lines.Add($"Model: {model.Model.Variables.Count} variables, " +
                              $"{model.Model.Constraints.Count} constraints.");
                }
                catch (Exception ex)
                {
                    lines.Add($"Model stats unavailable ({ex.GetType().Name}); " +
                              $"booleans={edgeBools.Count} (= edge count).");
                }

                lines.Add($"Status: {status}");
                lines.Add($"Solver wall time: {wall:F4}s");

                if (status != CpSolverStatus.Optimal && status != CpSolverStatus.Feasible)
                {
                    lines.Add("No placement returned.");
                    FlushAndEcho(lines);
                    return Result.Success;
                }

                long bxSol = solver.Value(Bx);
                long bySol = solver.Value(By);
                double bxIn = bxSol / Scale;
                double byIn = bySol / Scale;
                lines.Add($"B solved reference position: grid=({bxSol},{bySol}) inches=({bxIn:F3},{byIn:F3})");

                // --- Interlock signal: do A and B bounding boxes overlap? ---
                var polyBsolved = polyB.Translate(bxIn, byIn);
                var ba = polyA.BoundingBox;
                var bb = polyBsolved.BoundingBox;
                bool bboxOverlap =
                    ba.MinX < bb.MaxX && bb.MinX < ba.MaxX &&
                    ba.MinY < bb.MaxY && bb.MinY < ba.MaxY;
                lines.Add($"A bbox [{ba.MinX:F2},{ba.MinY:F2}]..[{ba.MaxX:F2},{ba.MaxY:F2}]");
                lines.Add($"B bbox [{bb.MinX:F2},{bb.MinY:F2}]..[{bb.MaxX:F2},{bb.MaxY:F2}]");
                lines.Add($"Bounding boxes OVERLAP: {bboxOverlap}  " +
                          $"(true + feasible => encoding permits interlock)");

                // --- Ground truth: do the actual polygons overlap? ---
                bool realOverlap = OverlapChecker.Overlaps(polyA, polyBsolved);
                lines.Add($"ACTUAL polygon overlap at solution: {realOverlap}  " +
                          (realOverlap
                            ? "<-- FALSE POSITIVE: concave-relaxation admitted a true overlap."
                            : "<-- genuine non-overlapping placement."));

                // --- Draw A and B at solved positions. ---
                int layA = GetOrCreateLayer(doc, "CPSAT_A", System.Drawing.Color.SteelBlue);
                int layB = GetOrCreateLayer(doc, "CPSAT_B", System.Drawing.Color.OrangeRed);
                doc.Objects.AddCurve(PolygonToCurve.ToCurve(polyA),
                    new ObjectAttributes { LayerIndex = layA });
                doc.Objects.AddCurve(PolygonToCurve.ToCurve(polyBsolved),
                    new ObjectAttributes { LayerIndex = layB });
                doc.Views.Redraw();
                lines.Add("Drew A on CPSAT_A, B (solved) on CPSAT_B.");

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
                RhinoApp.WriteLine("SeaNestCpSatNfp2 failed — see phase29_cpsat_nfp2.txt.");
                return Result.Failure;
            }
#else
            RhinoApp.WriteLine(
                "SeaNestCpSatNfp2 requires the .NET 7 (Rhino 8) build; " +
                "OR-Tools is not referenced for the net48 (Rhino 7) target.");
            return Result.Nothing;
#endif
        }

#if NET7_0
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

        private static void FlushAndEcho(List<string> lines)
        {
            foreach (var line in lines)
                RhinoApp.WriteLine(line);
            try
            {
                string path = Path.Combine(
                    System.Environment.GetFolderPath(
                        System.Environment.SpecialFolder.DesktopDirectory),
                    "phase29_cpsat_nfp2.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase29_cpsat_nfp2.txt: {ex.Message}");
            }
        }
#endif
    }
}
