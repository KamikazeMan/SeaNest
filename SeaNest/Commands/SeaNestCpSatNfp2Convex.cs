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
    // Phase 29 CP-SAT-on-frames spike, CORRECT (convex-decomposition) encoding.
    //
    // The naive SeaNestCpSatNfp2 encoded "B outside the NFP" as "B right of >=1
    // NFP edge" — exact only for convex polygons. For the concave NFP it was
    // over-permissive and produced a false positive (B in a reflex pocket,
    // actual polygons overlapping) despite a 2ms solve.
    //
    // Correct encoding: decompose the concave NFP outer contour into convex
    // pieces (here: ear-clipping triangulation — every triangle is convex,
    // simple and guaranteed correct, though more pieces than an optimal convex
    // partition). "B outside the whole NFP" == "B outside EVERY piece". Per
    // piece, "B outside" == OR over that piece's edges (B strictly right of
    // >=1 edge), reified. AND across pieces is implicit (each piece's OR is its
    // own AddBoolOr constraint, all must hold). This is mathematically exact.
    //
    // The pass/fail line is OverlapChecker.Overlaps at the solution: it MUST be
    // False now. The naive command is left intact for comparison.
    //
    // #if NET7_0-guarded (uses OR-Tools); net48 build gets a stub.
    public class SeaNestCpSatNfp2Convex : Command
    {
        public override string EnglishName => "SeaNestCpSatNfp2Convex";

        private const double Spacing = 0.25;
        private const double GridUnitInches = 0.05;
        private const double Scale = 1.0 / GridUnitInches; // 20 units/inch
        private const double SheetWidthInches = 240.0;
        private const double SheetHeightInches = 72.0;
        private const double TimeLimitSeconds = 60.0;

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
#if NET7_0
            var lines = new List<string>
            {
                $"Phase 29 CP-SAT NFP 2-frame CONVEX-DECOMP spike run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                $"Spacing: {Spacing:F3}\"  |  Grid: 1 unit = {GridUnitInches:F3}\" ({Scale:F0} units/inch)  |  " +
                $"Sheet: {SheetWidthInches:F0}x{SheetHeightInches:F0}\"  |  TimeLimit: {TimeLimitSeconds:F0}s",
                "Decomposition: ear-clipping triangulation (each triangle convex).",
                ""
            };

            try
            {
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

                IReadOnlyList<Polygon> nfp = NoFitPolygon.Compute(polyA, polyB, Spacing);
                if (nfp.Count == 0)
                {
                    lines.Add("NFP empty — frames do not interact.");
                    FlushAndEcho(lines);
                    return Result.Success;
                }

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
                    lines.Add("No CCW outer contour in NFP.");
                    FlushAndEcho(lines);
                    return Result.Failure;
                }
                if (holeCount > 0)
                    lines.Add($"WARNING: NFP has {holeCount} hole(s); spike decomposes only the outer contour.");

                int edgeCount = outer.Points.Count;
                lines.Add($"NFP outer contour: {edgeCount} edges/vertices.");

                // --- Convex decomposition: ear-clipping triangulation. ---
                var tris = EarClip(outer.Points, out bool stalled);
                if (stalled)
                    lines.Add("WARNING: ear-clipping stalled; remaining polygon fan-triangulated. " +
                              "Decomposition may be incomplete — trust the ACTUAL-overlap line below.");
                lines.Add($"Convex pieces (triangles): {tris.Count}");
                if (tris.Count == 0)
                {
                    lines.Add("Triangulation produced no pieces — cannot encode.");
                    FlushAndEcho(lines);
                    return Result.Failure;
                }

                // --- CP-SAT model. ---
                var model = new CpModel();

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

                // For each convex triangle: B outside it == OR over its 3 edges
                // (B strictly right of the directed CCW edge). AND across tris.
                int totalBooleans = 0;
                int degenerateTris = 0;
                foreach (var t in tris)
                {
                    // Ensure CCW so "interior is left, outside is right" holds.
                    var tri = EnsureCcw(t);
                    if (TriAreaAbs(tri) < 1e-9)
                    {
                        degenerateTris++;
                        continue; // zero-area sliver: no interior to exclude
                    }

                    var pts3 = new[] { tri.A, tri.B, tri.C };
                    var pieceBools = new List<ILiteral>(3);
                    for (int e = 0; e < 3; e++)
                    {
                        var vi = pts3[e];
                        var vj = pts3[(e + 1) % 3];
                        long xi = (long)Math.Round(vi.X * Scale);
                        long yi = (long)Math.Round(vi.Y * Scale);
                        long xj = (long)Math.Round(vj.X * Scale);
                        long yj = (long)Math.Round(vj.Y * Scale);
                        long dx = xj - xi;
                        long dy = yj - yi;
                        if (dx == 0 && dy == 0) continue;

                        long c = dy * xi - dx * yi;
                        long rhs = -1 - c; // cross <= -1  => strictly right (outside)

                        BoolVar be = model.NewBoolVar($"out_{totalBooleans}");
                        var lhs = LinearExpr.WeightedSum(
                            new LinearExpr[] { Bx, By }, new long[] { -dy, dx });
                        model.Add(lhs <= rhs).OnlyEnforceIf(be);
                        pieceBools.Add(be);
                        totalBooleans++;
                    }
                    if (pieceBools.Count > 0)
                        model.AddBoolOr(pieceBools); // outside this convex piece
                }
                lines.Add($"Total disjunction booleans: {totalBooleans} " +
                          $"(skipped {degenerateTris} degenerate triangle(s)).");

                // Objective: minimize combined bbox width in X.
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
                    lines.Add($"Model stats unavailable ({ex.GetType().Name}); " +
                              $"booleans={totalBooleans}.");
                }

                lines.Add($"Status: {status}");
                lines.Add($"Solver wall time: {wall:F4}s");

                if (status != CpSolverStatus.Optimal && status != CpSolverStatus.Feasible)
                {
                    lines.Add("No placement returned. (If Infeasible, the decomposition may " +
                              "over-constrain — check degenerate-triangle handling.)");
                    FlushAndEcho(lines);
                    return Result.Success;
                }

                long bxSol = solver.Value(Bx);
                long bySol = solver.Value(By);
                double bxIn = bxSol / Scale;
                double byIn = bySol / Scale;
                lines.Add($"B solved reference position: grid=({bxSol},{bySol}) inches=({bxIn:F3},{byIn:F3})");

                var polyBsolved = polyB.Translate(bxIn, byIn);
                var ba = polyA.BoundingBox;
                var bb = polyBsolved.BoundingBox;
                bool bboxOverlap =
                    ba.MinX < bb.MaxX && bb.MinX < ba.MaxX &&
                    ba.MinY < bb.MaxY && bb.MinY < ba.MaxY;
                lines.Add($"Bounding boxes OVERLAP: {bboxOverlap}  (true => layout interlocks bbox-wise)");

                bool realOverlap = OverlapChecker.Overlaps(polyA, polyBsolved);
                lines.Add($"ACTUAL polygon overlap at solution: {realOverlap}  " +
                          (realOverlap
                            ? "<-- FAIL: encoding still admits a real overlap (decomposition incomplete/bug)."
                            : "<-- PASS: valid non-overlapping placement (correct encoding)."));

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
                RhinoApp.WriteLine("SeaNestCpSatNfp2Convex failed — see phase29_cpsat_nfp2convex.txt.");
                return Result.Failure;
            }
#else
            RhinoApp.WriteLine(
                "SeaNestCpSatNfp2Convex requires the .NET 7 (Rhino 8) build; " +
                "OR-Tools is not referenced for the net48 (Rhino 7) target.");
            return Result.Nothing;
#endif
        }

#if NET7_0
        private struct Tri { public Point2D A, B, C; }

        // Ear-clipping triangulation of a simple CCW polygon. Returns convex
        // triangles. Sets stalled=true and fan-triangulates the remainder if no
        // ear is found (non-simple/degenerate input); the ACTUAL-overlap check
        // is the ground truth in that case.
        private static List<Tri> EarClip(IReadOnlyList<Point2D> ptsIn, out bool stalled)
        {
            stalled = false;
            var result = new List<Tri>();
            int n = ptsIn.Count;
            if (n < 3) return result;

            var idx = new List<int>(n);
            for (int i = 0; i < n; i++) idx.Add(i);

            int guard = 0;
            int maxIter = n * n + 16;
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

                    // Convex (CCW) vertex test: signed area of (a,b,c) > 0.
                    if (SignedArea2(a, b, c) <= 0) continue;

                    // No other polygon vertex inside this candidate ear.
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
            {
                result.Add(new Tri { A = ptsIn[idx[0]], B = ptsIn[idx[1]], C = ptsIn[idx[2]] });
            }
            else if (idx.Count > 3)
            {
                // Fallback fan (only on stall): triangles may be non-convex; the
                // overlap check will reveal any resulting invalidity.
                for (int i = 1; i + 1 < idx.Count; i++)
                    result.Add(new Tri { A = ptsIn[idx[0]], B = ptsIn[idx[i]], C = ptsIn[idx[i + 1]] });
            }
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
                    "phase29_cpsat_nfp2convex.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase29_cpsat_nfp2convex.txt: {ex.Message}");
            }
        }
#endif
    }
}
