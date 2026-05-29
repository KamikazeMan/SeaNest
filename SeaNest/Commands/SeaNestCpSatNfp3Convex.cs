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
    // Phase 29 CP-SAT-on-frames spike: THREE frames, convex-decomposition.
    //
    // Measures the 2->3 frame solve-time curve (2 frames = 1 pair; 3 frames =
    // 3 pairs: 0-1, 0-2, 1-2). That curve, not the boolean count, decides
    // whether 5 frames is viable.
    //
    // Model: frame 0 is FIXED at its flattened coords (the reference, drawn
    // as-is). Frames 1 and 2 are placed by the solver via position variables
    // (B1x,B1y) and (B2x,B2y). This removes global translational symmetry.
    //
    // No-overlap per pair = "moving point outside the convex-decomposed NFP"
    // (ear-clip triangulation, "outside every triangle", reified — the encoding
    // validated by the 2-frame convex spike). The point differs per pair:
    //   pair 0-1 (fixed-moving): P = (B1x, B1y)            outside NFP(poly0,poly1)
    //   pair 0-2 (fixed-moving): P = (B2x, B2y)            outside NFP(poly0,poly2)
    //   pair 1-2 (MOVING-MOVING): P = (B2x-B1x, B2y-B1y)   outside NFP(poly1,poly2)
    // The moving-moving pair constrains the RELATIVE position (difference of
    // the two variable sets), derived from:
    //   (poly1+B1) ∩ (poly2+B2) != empty  <=>  (B2-B1) ∈ NFP(poly1,poly2)
    // (shift both by -B1; poly1 returns to its flatten frame, exactly the frame
    // NoFitPolygon.Compute used). Encoded via per-variable coefficient vectors
    // so the half-planes act on the difference, not absolute positions.
    //
    // #if NET7_0-guarded (uses OR-Tools).
    public class SeaNestCpSatNfp3Convex : Command
    {
        public override string EnglishName => "SeaNestCpSatNfp3Convex";

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
                $"Phase 29 CP-SAT NFP 3-frame CONVEX-DECOMP spike run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                $"Spacing: {Spacing:F3}\"  |  Grid: 1 unit = {GridUnitInches:F3}\" ({Scale:F0} units/inch)  |  " +
                $"Sheet: {SheetWidthInches:F0}x{SheetHeightInches:F0}\"  |  TimeLimit: {TimeLimitSeconds:F0}s",
                "Model: frame 0 fixed at flatten coords; frames 1,2 solver-placed. " +
                "Pairs 0-1,0-2 absolute; pair 1-2 relative (B2-B1).",
                ""
            };

            try
            {
                var go = new GetObject();
                go.SetCommandPrompt("Select THREE hull frames: frame 0 (fixed) first, then 1, then 2");
                go.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
                go.SubObjectSelect = false;
                go.GetMultiple(3, 3);
                if (go.CommandResult() != Result.Success)
                    return go.CommandResult();

                var polys = new Polygon[3];
                for (int i = 0; i < 3; i++)
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

                // Position vars for frames 1 and 2 (frame 0 fixed). Order:
                // [B1x, B1y, B2x, B2y].
                var vars = new IntVar[4];
                if (!MakePosVars(model, polys[1], "B1", out vars[0], out vars[1], lines)) { FlushAndEcho(lines); return Result.Failure; }
                if (!MakePosVars(model, polys[2], "B2", out vars[2], out vars[3], lines)) { FlushAndEcho(lines); return Result.Failure; }

                // Pair definitions: (i,j, PxCoef[4], PyCoef[4]).
                // Coef vectors map [B1x,B1y,B2x,B2y] -> the pair's moving point.
                var pairs = new (int I, int J, int[] Px, int[] Py)[]
                {
                    (0, 1, new[]{ 1,0,0,0 }, new[]{ 0,1,0,0 }),   // P = B1
                    (0, 2, new[]{ 0,0,1,0 }, new[]{ 0,0,0,1 }),   // P = B2
                    (1, 2, new[]{ -1,0,1,0 }, new[]{ 0,-1,0,1 }),  // P = B2 - B1
                };

                int grandBooleans = 0;
                foreach (var pr in pairs)
                {
                    var nfp = NoFitPolygon.Compute(polys[pr.I], polys[pr.J], Spacing);
                    Polygon outer = LargestCcw(nfp, out int holeCount);
                    if (outer == null)
                    {
                        lines.Add($"Pair {pr.I}-{pr.J}: NFP empty or no CCW outer — frames do not interact; skipped.");
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

                // Objective: minimize combined bbox width in X across all 3 frames.
                // Frame 0 fixed -> constant extents. Frames 1,2 -> var + offset.
                long wide = (long)Math.Round((SheetWidthInches * 4) * Scale);
                IntVar maxX = model.NewIntVar(-wide, wide, "maxX");
                IntVar minX = model.NewIntVar(-wide, wide, "minX");

                long f0MinX = (long)Math.Round(polys[0].BoundingBox.MinX * Scale);
                long f0MaxX = (long)Math.Round(polys[0].BoundingBox.MaxX * Scale);
                model.Add(maxX >= f0MaxX);
                model.Add(minX <= f0MinX);
                // frame 1
                model.Add(maxX >= vars[0] + (long)Math.Round(polys[1].BoundingBox.MaxX * Scale));
                model.Add(minX <= vars[0] + (long)Math.Round(polys[1].BoundingBox.MinX * Scale));
                // frame 2
                model.Add(maxX >= vars[2] + (long)Math.Round(polys[2].BoundingBox.MaxX * Scale));
                model.Add(minX <= vars[2] + (long)Math.Round(polys[2].BoundingBox.MinX * Scale));
                model.Minimize(maxX - minX);

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

                double b1x = solver.Value(vars[0]) / Scale, b1y = solver.Value(vars[1]) / Scale;
                double b2x = solver.Value(vars[2]) / Scale, b2y = solver.Value(vars[3]) / Scale;
                lines.Add($"Frame 0 (fixed): at flatten coords.");
                lines.Add($"Frame 1 solved: ({b1x:F3},{b1y:F3})");
                lines.Add($"Frame 2 solved: ({b2x:F3},{b2y:F3})");

                var p0 = polys[0];
                var p1 = polys[1].Translate(b1x, b1y);
                var p2 = polys[2].Translate(b2x, b2y);

                // Ground-truth overlap for ALL THREE pairs. All must be False.
                bool o01 = OverlapChecker.Overlaps(p0, p1);
                bool o02 = OverlapChecker.Overlaps(p0, p2);
                bool o12 = OverlapChecker.Overlaps(p1, p2);
                lines.Add($"ACTUAL overlap 0-1: {o01}");
                lines.Add($"ACTUAL overlap 0-2: {o02}");
                lines.Add($"ACTUAL overlap 1-2: {o12}");
                bool allClear = !o01 && !o02 && !o12;
                lines.Add(allClear
                    ? "PASS: all three pairs non-overlapping (correct encoding)."
                    : "FAIL: at least one pair overlaps (decomposition/encoding issue).");

                // Interlock signal: any bbox overlaps among the placed frames.
                lines.Add($"bbox overlap 0-1: {BBoxOverlap(p0, p1)}, " +
                          $"0-2: {BBoxOverlap(p0, p2)}, 1-2: {BBoxOverlap(p1, p2)}");

                int l0 = GetOrCreateLayer(doc, "CPSAT_F0", System.Drawing.Color.SteelBlue);
                int l1 = GetOrCreateLayer(doc, "CPSAT_F1", System.Drawing.Color.OrangeRed);
                int l2 = GetOrCreateLayer(doc, "CPSAT_F2", System.Drawing.Color.SeaGreen);
                doc.Objects.AddCurve(PolygonToCurve.ToCurve(p0), new ObjectAttributes { LayerIndex = l0 });
                doc.Objects.AddCurve(PolygonToCurve.ToCurve(p1), new ObjectAttributes { LayerIndex = l1 });
                doc.Objects.AddCurve(PolygonToCurve.ToCurve(p2), new ObjectAttributes { LayerIndex = l2 });
                doc.Views.Redraw();
                lines.Add("Drew frames on CPSAT_F0 / CPSAT_F1 / CPSAT_F2.");

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
                RhinoApp.WriteLine("SeaNestCpSatNfp3Convex failed — see phase29_cpsat_nfp3convex.txt.");
                return Result.Failure;
            }
#else
            RhinoApp.WriteLine(
                "SeaNestCpSatNfp3Convex requires the .NET 7 (Rhino 8) build; " +
                "OR-Tools is not referenced for the net48 (Rhino 7) target.");
            return Result.Nothing;
#endif
        }

#if NET7_0
        // Position vars for a moving frame so its bbox stays on-sheet.
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

        // For each convex triangle: moving point P (given by Px/Py coef vectors
        // over [B1x,B1y,B2x,B2y]) must be OUTSIDE the triangle == strictly RIGHT
        // of >=1 of its CCW edges. Reified per edge, AddBoolOr per triangle.
        // Returns the number of booleans created for this pair.
        private static int EncodePairOutside(
            CpModel model, IntVar[] vars, int[] px, int[] py,
            IReadOnlyList<Tri> tris, ref int boolCounter)
        {
            int created = 0;
            var exprVars = new LinearExpr[] { vars[0], vars[1], vars[2], vars[3] };

            foreach (var raw in tris)
            {
                var t = EnsureCcw(raw);
                if (TriAreaAbs(t) < 1e-9) continue; // sliver: no interior

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

                    // cross(edge, P - vi) <= -1  (strictly right / outside).
                    // cross = (-dy)*Px + dx*Py + (dy*xi - dx*yi).
                    // Px = sum px[k]*var[k], Py = sum py[k]*var[k], so the
                    // coefficient on var[k] is (-dy)*px[k] + dx*py[k].
                    var coefs = new long[4];
                    for (int k = 0; k < 4; k++)
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

        private static bool BBoxOverlap(Polygon a, Polygon b)
        {
            var ba = a.BoundingBox; var bb = b.BoundingBox;
            return ba.MinX < bb.MaxX && bb.MinX < ba.MaxX &&
                   ba.MinY < bb.MaxY && bb.MinY < ba.MaxY;
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
                    if (SignedArea2(a, b, c) <= 0) continue; // not a convex vertex

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
                    "phase29_cpsat_nfp3convex.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase29_cpsat_nfp3convex.txt: {ex.Message}");
            }
        }
#endif
    }
}
