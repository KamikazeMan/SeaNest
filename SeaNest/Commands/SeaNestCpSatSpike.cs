using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using Rhino;
using Rhino.Commands;
#if NET7_0
using Google.OrTools.Sat;
#endif

namespace SeaNest.Commands
{
    // Phase 29 CP-SAT feasibility spike (THROWAWAY).
    //
    // Gating question: does Google.OrTools load and run inside a Rhino 8 /
    // .NET 7 plugin? This command models 3 axis-aligned rectangles on a
    // 240x72 sheet with CP-SAT's AddNoOverlap2D, solves, and reports the
    // placements + solve time to the command line and to a Desktop file. It
    // deliberately avoids concave polygons, rotations, and NFP integration —
    // it only confirms (a) OR-Tools loads, (b) a trivial 2D no-overlap model
    // solves, (c) results come back usable.
    public class SeaNestCpSatSpike : Command
    {
        public override string EnglishName => "SeaNestCpSatSpike";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
#if NET7_0
            var lines = new List<string>
            {
                $"Phase 29 CP-SAT spike run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                "Sheet: 240 x 72. Rectangles (w x h): 100x50, 80x40, 60x60.",
                ""
            };

            const int sheetW = 240;
            const int sheetH = 72;
            var sizes = new (int W, int H)[] { (100, 50), (80, 40), (60, 60) };

            CpSolverStatus status;
            double wallTimeSec;
            var sw = Stopwatch.StartNew();

            try
            {
                var model = new CpModel();

                var xs = new IntVar[sizes.Length];
                var ys = new IntVar[sizes.Length];
                var xIntervals = new IntervalVar[sizes.Length];
                var yIntervals = new IntervalVar[sizes.Length];

                for (int i = 0; i < sizes.Length; i++)
                {
                    int w = sizes[i].W;
                    int h = sizes[i].H;

                    // Sheet-bounds constraints are encoded directly in the
                    // variable domains: a rectangle's origin can range from 0
                    // to sheet_dim - rect_dim, keeping it fully on the sheet.
                    xs[i] = model.NewIntVar(0, sheetW - w, $"x{i}");
                    ys[i] = model.NewIntVar(0, sheetH - h, $"y{i}");

                    xIntervals[i] = model.NewFixedSizeIntervalVar(xs[i], w, $"xiv{i}");
                    yIntervals[i] = model.NewFixedSizeIntervalVar(ys[i], h, $"yiv{i}");
                }

                // Pairwise non-overlap for all rectangles in one 2D constraint.
                NoOverlap2dConstraint noOverlap = model.AddNoOverlap2D();
                for (int i = 0; i < sizes.Length; i++)
                    noOverlap.AddRectangle(xIntervals[i], yIntervals[i]);

                var solver = new CpSolver();
                status = solver.Solve(model);
                wallTimeSec = solver.WallTime();

                lines.Add($"Status: {status}");
                lines.Add($"Solver wall time: {wallTimeSec:F4}s");

                if (status == CpSolverStatus.Optimal || status == CpSolverStatus.Feasible)
                {
                    for (int i = 0; i < sizes.Length; i++)
                    {
                        long x = solver.Value(xs[i]);
                        long y = solver.Value(ys[i]);
                        lines.Add($"  Rect {i} ({sizes[i].W}x{sizes[i].H}): x={x}, y={y}");
                    }
                }
                else
                {
                    lines.Add("  No feasible placement returned.");
                }
            }
            catch (Exception ex)
            {
                sw.Stop();
                // Most likely failure mode for the gating question: the OR-Tools
                // managed assembly or its native binaries fail to load in the
                // Rhino plugin context (TypeInitializationException /
                // DllNotFoundException / BadImageFormatException).
                lines.Add($"EXCEPTION ({ex.GetType().Name}): {ex.Message}");
                if (ex.InnerException != null)
                    lines.Add($"  Inner ({ex.InnerException.GetType().Name}): {ex.InnerException.Message}");

                FlushAndEcho(lines);
                RhinoApp.WriteLine("SeaNestCpSatSpike: OR-Tools failed to load/run — see phase29_spike.txt.");
                return Result.Failure;
            }

            sw.Stop();
            lines.Add($"Total command time (incl. model build): {sw.Elapsed.TotalSeconds:F4}s");

            FlushAndEcho(lines);
            return Result.Success;
#else
            RhinoApp.WriteLine(
                "SeaNestCpSatSpike requires the .NET 7 (Rhino 8) build; " +
                "OR-Tools is not referenced for the net48 (Rhino 7) target.");
            return Result.Nothing;
#endif
        }

#if NET7_0
        private static void FlushAndEcho(List<string> lines)
        {
            foreach (var line in lines)
                RhinoApp.WriteLine(line);

            try
            {
                string path = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory),
                    "phase29_spike.txt");
                File.WriteAllLines(path, lines);
                RhinoApp.WriteLine($"Wrote {path}");
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"Could not write phase29_spike.txt: {ex.Message}");
            }
        }
#endif
    }
}
