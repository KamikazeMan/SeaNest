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
    // Phase 29 CP-SAT-on-frames spike: FOUR frames, convex-decomposition.
    //
    // Third data point on the solve-time curve (2 frames=1 pair=10ms,
    // 3 frames=3 pairs=212ms, 4 frames=6 pairs=?) to fit the growth and
    // project 5 frames (10 pairs) before committing.
    //
    // Model: frame 0 FIXED at flatten coords; frames 1,2,3 solver-placed via
    // [B1x,B1y,B2x,B2y,B3x,B3y]. 6 pairs. Pairs with frame 0 (0-1,0-2,0-3) use
    // absolute position; solved-solved pairs (1-2,1-3,2-3) use the relative
    // position (B_j - B_i) outside NFP(poly_i,poly_j) — the coefficient-vector
    // encoder from the 3-frame command, generalized to N variables.
    //
    // IMPROVED OBJECTIVE (loose-fan fix): minimize half-perimeter of the
    // combined bounding box = (maxX-minX) + (maxY-minY). True bbox AREA is a
    // nonlinear product of two variables and can't be a linear CP-SAT
    // objective; half-perimeter is the clean linear surrogate that penalizes
    // spread in BOTH dimensions and pulls the frames into a tight stack rather
    // than a fan.
    //
    // #if NET7_0-guarded (uses OR-Tools).
    public class SeaNestCpSatNfp4Convex : Command
    {
        public override string EnglishName => "SeaNestCpSatNfp4Convex";

        private const double Spacing = 0.25;
        private const double GridUnitInches = 0.05;
        private const double Scale = 1.0 / GridUnitInches; // 20 units/inch
        private const double SheetWidthInches = 240.0;
        private const double SheetHeightInches = 72.0;
        private const double TimeLimitSeconds = 120.0;

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
#if NET7_0
            var lines = new List<string>
            {
                $"Phase 29 CP-SAT NFP 4-frame CONVEX-DECOMP spike run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                $"Spacing: {Spacing:F3}\"  |  Grid: 1 unit = {GridUnitInches:F3}\" ({Scale:F0} units/inch)  |  " +
                $"Sheet: {SheetWidthInches:F0}x{SheetHeightInches:F0}\"  |  TimeLimit: {TimeLimitSeconds:F0}s",
                "Model: frame 0 fixed; frames 1,2,3 solved. 6 pairs. " +
                "Objective: minimize (maxX-minX)+(maxY-minY) [half-perimeter, tightness].",
                ""
            };

            try
            {
                var go = new GetObject();
                go.SetCommandPrompt("Select FOUR hull frames: frame 0 (fixed) first, then 1,2,3");
                go.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
                go.SubObjectSelect = false;
                go.GetMultiple(4, 4);
                if (go.CommandResult() != Result.Success)
                    return go.CommandResult();

                var polys = new Polygon[4];
                for (int i = 0; i < 4; i++)
                {
                    Brep brep = ToBrep(go.Object(i));
                    if (brep == null)
                    {
                        lines.Add($"Could not obtain Brep geometry for selection {i}.");
                        FlushAndEcho(lines);
                        return Result.Failure;
                    }
                    var flat = BrepFlattener.Flatten(brep, doc);
                    if (flat?.OuterPolygon == null)
                    {
                        lines.Add($"BrepFlattener returned no OuterPolygon for selection {i}.");
                        FlushAndEcho(lines);
                        return Result.Failure;
                    }
                    polys[i] = flat.OuterPolygon;
                }

                var model = new CpModel();

                // Position vars for frames 1,2,3: [B1x,B1y,B2x,B2y,B3x,B3y].
                var vars = new IntVar[6];
                if (!MakePosVars(model, polys[1], "B1", out vars[0], out vars[1], lines)) { FlushAndEcho(lines); return Result.Failure; }
                if (!MakePosVars(model, polys[2], "B2", out vars[2], out vars[3], lines)) { FlushAndEcho(lines); return Result.Failure; }
                if (!MakePosVars(model, polys[3], "B3", out vars[4], out vars[5], lines)) { FlushAndEcho(lines); return Result.Failure; }

                // Pair coef vectors over [B1x,B1y,B2x,B2y,B3x,B3y].
                // pos_0=(0,0) fixed; pos_k var index: x=2(k-1), y=2(k-1)+1.
                var pairs = new (int I, int J, int[] Px, int[] Py)[]
                {
                    (0, 1, V(6, (0,1)),            V(6, (1,1))),             // P=B1
                    (0, 2, V(6, (2,1)),            V(6, (3,1))),             // P=B2
                    (0, 3, V(6, (4,1)),            V(6, (5,1))),             // P=B3
                    (1, 2, V(6, (0,-1),(2,1)),     V(6, (1,-1),(3,1))),      // P=B2-B1
                    (1, 3, V(6, (0,-1),(4,1)),     V(6, (1,-1),(5,1))),      // P=B3-B1
                    (2, 3, V(6, (2,-1),(4,1)),     V(6, (3,-1),(5,1))),      // P=B3-B2
                };

                int grandBooleans = 0;
                foreach (var pr in pairs)
                {
                    var nfp = NoFitPolygon.Compute(polys[pr.I], polys[pr.J], Spacing);
                    Polygon outer = LargestCcw(nfp, out int holeCount);
                    if (outer == null)
                    {
                        lines.Add($"Pair {pr.I}-{pr.J}: NFP empty/no CCW outer — frames do not interact; skipped.");
                        continue;
                    }
                    if (holeCount > 0)
                        lines.Add($"Pair {pr.I}-{pr.J}: WARNING {holeCount} NFP hole(s) ignored (outer only).");

                    int edgeCount = outer.Points.Count;
                    var tris = EarClip(outer.Points, out bool stalled);
                    if (stalled)
                        lines.Add($"Pair {pr.I}-{pr.J}: WARNING ear-clip stalled; remainder fan-triangulated.");

                    int pairBooleans = EncodePairOutside(model, vars, pr.Px, pr.Py, tris, ref grandBooleans);
                    lines.Add($"Pair {pr.I}-{pr.J}: NFP edges={edgeCount}, triangles={tris.Count}, booleans={pairBooleans}");
                }
                lines.Add($"TOTAL disjunction booleans: {grandBooleans}");

                // --- Tightness objective: half-perimeter of combined bbox. ---
                long wide = (long)Math.Round((SheetWidthInches * 4) * Scale);
                IntVar maxX = model.NewIntVar(-wide, wide, "maxX");
                IntVar minX = model.NewIntVar(-wide, wide, "minX");
                IntVar maxY = model.NewIntVar(-wide, wide, "maxY");
                IntVar minY = model.NewIntVar(-wide, wide, "minY");

                // Frame 0 fixed extents (constants).
                AddExtent(model, maxX, minX, maxY, minY, null, null, polys[0]);
                // Frames 1,2,3 = var + offset.
                AddExtent(model, maxX, minX, maxY, minY, vars[0], vars[1], polys[1]);
                AddExtent(model, maxX, minX, maxY, minY, vars[2], vars[3], polys[2]);
                AddExtent(model, maxX, minX, maxY, minY, vars[4], vars[5], polys[3]);

                model.Minimize(maxX - minX + maxY - minY);

                var solver = new CpSolver();
                solver.StringParameters =
                    $"max_time_in_seconds:{TimeLimitSeconds},random_seed:1,num_search_workers:1";
                CpSolverStatus status = solver.Solve(model);
                double wall = solver.WallTime();

                try
                {
                    lines.Add($"Model: {model.Model.Variables.Count} variables, " +
                              $"{model.Model.Constraints.Count} constraints.");
                }
                catch (Exception ex)
                {
                    lines.Add($"Model stats unavailable ({ex.GetType().Name}); booleans={grandBooleans}.");
                }

                lines.Add($"Status: {status}");
                lines.Add($"Solver wall time: {wall:F4}s");

                if (status != CpSolverStatus.Optimal && status != CpSolverStatus.Feasible)
                {
                    lines.Add("No placement returned.");
                    FlushAndEcho(lines);
                    return Result.Success;
                }

                double[] sx = new double[4], sy = new double[4];
                sx[0] = 0; sy[0] = 0;
                sx[1] = solver.Value(vars[0]) / Scale; sy[1] = solver.Value(vars[1]) / Scale;
                sx[2] = solver.Value(vars[2]) / Scale; sy[2] = solver.Value(vars[3]) / Scale;
                sx[3] = solver.Value(vars[4]) / Scale; sy[3] = solver.Value(vars[5]) / Scale;

                var placed = new Polygon[4];
                placed[0] = polys[0];
                for (int i = 1; i < 4; i++) placed[i] = polys[i].Translate(sx[i], sy[i]);

                lines.Add("Frame 0 (fixed): at flatten coords.");
                for (int i = 1; i < 4; i++)
                    lines.Add($"Frame {i} solved: ({sx[i]:F3},{sy[i]:F3})");

                // Ground truth: all 6 pairs must be non-overlapping.
                bool allClear = true;
                foreach (var pr in pairs)
                {
                    bool ov = OverlapChecker.Overlaps(placed[pr.I], placed[pr.J]);
                    lines.Add($"ACTUAL overlap {pr.I}-{pr.J}: {ov}");
                    if (ov) allClear = false;
                }
                lines.Add(allClear
                    ? "PASS: all six pairs non-overlapping."
                    : "FAIL: at least one pair overlaps.");

                // Tightness report: combined bbox extents/area at the solution.
                double gMinX = double.MaxValue, gMinY = double.MaxValue;
                double gMaxX = double.MinValue, gMaxY = double.MinValue;
                foreach (var p in placed)
                {
                    var b = p.BoundingBox;
                    gMinX = Math.Min(gMinX, b.MinX); gMinY = Math.Min(gMinY, b.MinY);
                    gMaxX = Math.Max(gMaxX, b.MaxX); gMaxY = Math.Max(gMaxY, b.MaxY);
                }
                double w = gMaxX - gMinX, h = gMaxY - gMinY;
                lines.Add($"Combined bbox: {w:F2} x {h:F2} = {w * h:F1} sq in " +
                          $"(half-perimeter {w + h:F2}).");

                int[] layers =
                {
                    GetOrCreateLayer(doc, "CPSAT_F0", System.Drawing.Color.SteelBlue),
                    GetOrCreateLayer(doc, "CPSAT_F1", System.Drawing.Color.OrangeRed),
                    GetOrCreateLayer(doc, "CPSAT_F2", System.Drawing.Color.SeaGreen),
                    GetOrCreateLayer(doc, "CPSAT_F3", System.Drawing.Color.MediumPurple),
                };
                for (int i = 0; i < 4; i++)
                    doc.Objects.AddCurve(PolygonToCurve.ToCurve(placed[i]),
                        new ObjectAttributes { LayerIndex = layers[i] });
                doc.Views.Redraw();
                lines.Add("Drew frames on CPSAT_F0..F3.");

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
                RhinoApp.WriteLine("SeaNestCpSatNfp4Convex failed — see phase29_cpsat_nfp4convex.txt.");
                return Result.Failure;
            }
#else
            RhinoApp.WriteLine(
                "SeaNestCpSatNfp4Convex requires the .NET 7 (Rhino 8) build; " +
                "OR-Tools is not referenced for the net48 (Rhino 7) target.");
            return Result.Nothing;
#endif
        }

#if NET7_0
        // Build a length-n coefficient vector with the given (index,value) pairs.
        private static int[] V(int n, params (int Idx, int Val)[] entries)
        {
            var a = new int[n];
            foreach (var e in entries) a[e.Idx] = e.Val;
            return a;
        }

        // Constrain the four global extent vars to cover one frame's bbox.
        // For the fixed frame pass vx=vy=null (constant offsets); for solved
        // frames pass its position vars.
        private static void AddExtent(
            CpModel model, IntVar maxX, IntVar minX, IntVar maxY, IntVar minY,
            IntVar vx, IntVar vy, Polygon poly)
        {
            var bb = poly.BoundingBox;
            long oMinX = (long)Math.Round(bb.MinX * Scale);
            long oMaxX = (long)Math.Round(bb.MaxX * Scale);
            long oMinY = (long)Math.Round(bb.MinY * Scale);
            long oMaxY = (long)Math.Round(bb.MaxY * Scale);

            if (vx == null)
            {
                model.Add(maxX >= oMaxX);
                model.Add(minX <= oMinX);
                model.Add(maxY >= oMaxY);
                model.Add(minY <= oMinY);
            }
            else
            {
                model.Add(maxX >= vx + oMaxX);
                model.Add(minX <= vx + oMinX);
                model.Add(maxY >= vy + oMaxY);
                model.Add(minY <= vy + oMinY);
            }
        }

        private static bool MakePosVars(
            CpModel model, Polygon poly, string name,
            out IntVar vx, out IntVar vy, List<string> lines)
        {
            var bb = poly.BoundingBox;
            long lox = (long)Math.Ceiling((0.0 - bb.MinX) * Scale);
            long hix = (long)Math.Floor((SheetWidthInches - bb.MaxX) * Scale);
            long loy = (long)Math.Ceiling((0.0 - bb.MinY) * Scale);
            long hiy = (long)Math.Floor((SheetHeightInches - bb.MaxY) * Scale);
            if (lox > hix || loy > hiy)
            {
                lines.Add($"{name} does not fit on sheet: x[{lox},{hix}] y[{loy},{hiy}] (grid).");
                vx = null; vy = null;
                return false;
            }
            vx = model.NewIntVar(lox, hix, name + "x");
            vy = model.NewIntVar(loy, hiy, name + "y");
            return true;
        }

        // Generalized: moving point P = (sum px[k]*var[k], sum py[k]*var[k])
        // must be OUTSIDE every convex triangle (strictly right of >=1 CCW edge).
        private static int EncodePairOutside(
            CpModel model, IntVar[] vars, int[] px, int[] py,
            IReadOnlyList<Tri> tris, ref int boolCounter)
        {
            int created = 0;
            int n = vars.Length;
            var exprVars = new LinearExpr[n];
            for (int k = 0; k < n; k++) exprVars[k] = vars[k];

            foreach (var raw in tris)
            {
                var t = EnsureCcw(raw);
                if (TriAreaAbs(t) < 1e-9) continue;

                var pts3 = new[] { t.A, t.B, t.C };
                var pieceBools = new List<ILiteral>(3);
                for (int e = 0; e < 3; e++)
                {
                    var vi = pts3[e];
                    var vj = pts3[(e + 1) % 3];
                    long xi = (long)Math.Round(vi.X * Scale);
                    long yi = (long)Math.Round(vi.Y * Scale);
                    long xj = (long)Math.Round(vj.X * Scale);
                    long yj = (long)Math.Round(vj.Y * Scale);
                    long dx = xj - xi, dy = yj - yi;
                    if (dx == 0 && dy == 0) continue;

                    var coefs = new long[n];
                    for (int k = 0; k < n; k++)
                        coefs[k] = (-dy) * px[k] + dx * py[k];

                    long c = dy * xi - dx * yi;
                    long rhs = -1 - c;

                    BoolVar be = model.NewBoolVar($"out_{boolCounter}");
                    boolCounter++;
                    created++;
                    var lhs = LinearExpr.WeightedSum(exprVars, coefs);
                    model.Add(lhs <= rhs).OnlyEnforceIf(be);
                    pieceBools.Add(be);
                }
                if (pieceBools.Count > 0)
                    model.AddBoolOr(pieceBools);
            }
            return created;
        }

        private static Polygon LargestCcw(IReadOnlyList<Polygon> nfp, out int holeCount)
        {
            holeCount = 0;
            Polygon outer = null;
            foreach (var p in nfp)
            {
                if (p.IsCounterClockwise)
                {
                    if (outer == null || p.AbsoluteArea > outer.AbsoluteArea) outer = p;
                }
                else holeCount++;
            }
            return outer;
        }

        private struct Tri { public Point2D A, B, C; }

        private static List<Tri> EarClip(IReadOnlyList<Point2D> ptsIn, out bool stalled)
        {
            stalled = false;
            var result = new List<Tri>();
            int n = ptsIn.Count;
            if (n < 3) return result;

            var idx = new List<int>(n);
            for (int i = 0; i < n; i++) idx.Add(i);

            int guard = 0, maxIter = n * n + 16;
            while (idx.Count > 3 && guard++ < maxIter)
            {
                bool earFound = false;
                int count = idx.Count;
                for (int i = 0; i < count; i++)
                {
                    int ip = idx[(i - 1 + count) % count];
                    int ic = idx[i];
                    int inx = idx[(i + 1) % count];
                    Point2D a = ptsIn[ip], b = ptsIn[ic], c = ptsIn[inx];
                    if (SignedArea2(a, b, c) <= 0) continue;

                    bool anyInside = false;
                    for (int j = 0; j < count; j++)
                    {
                        int vj = idx[j];
                        if (vj == ip || vj == ic || vj == inx) continue;
                        if (PointInTriInclusive(ptsIn[vj], a, b, c)) { anyInside = true; break; }
                    }
                    if (anyInside) continue;

                    result.Add(new Tri { A = a, B = b, C = c });
                    idx.RemoveAt(i);
                    earFound = true;
                    break;
                }
                if (!earFound) { stalled = true; break; }
            }

            if (idx.Count == 3)
                result.Add(new Tri { A = ptsIn[idx[0]], B = ptsIn[idx[1]], C = ptsIn[idx[2]] });
            else if (idx.Count > 3)
                for (int i = 1; i + 1 < idx.Count; i++)
                    result.Add(new Tri { A = ptsIn[idx[0]], B = ptsIn[idx[i]], C = ptsIn[idx[i + 1]] });

            return result;
        }

        private static double SignedArea2(Point2D a, Point2D b, Point2D c)
            => (b.X - a.X) * (c.Y - a.Y) - (c.X - a.X) * (b.Y - a.Y);

        private static double TriAreaAbs(Tri t)
            => Math.Abs(SignedArea2(t.A, t.B, t.C)) * 0.5;

        private static Tri EnsureCcw(Tri t)
            => SignedArea2(t.A, t.B, t.C) >= 0 ? t : new Tri { A = t.A, B = t.C, C = t.B };

        private static bool PointInTriInclusive(Point2D p, Point2D a, Point2D b, Point2D c)
        {
            double d1 = SignedArea2(p, a, b);
            double d2 = SignedArea2(p, b, c);
            double d3 = SignedArea2(p, c, a);
            bool hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
            bool hasPos = d1 > 0 || d2 > 0 || d3 > 0;
            return !(hasNeg && hasPos);
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

        private static void FlushAndEcho(List<string> lines)
        {
            foreach (var line in lines)
                RhinoApp.WriteLine(line);
            try
            {
                string path = Path.Combine(
                    System.Environment.GetFolderPath(
                        System.Environment.SpecialFolder.DesktopDirectory),
                    "phase29_cpsat_nfp4convex.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase29_cpsat_nfp4convex.txt: {ex.Message}");
            }
        }
#endif
    }
}
