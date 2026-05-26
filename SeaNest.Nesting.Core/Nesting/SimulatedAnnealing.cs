using System;
using System.Collections.Generic;
using System.Diagnostics;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Smart order optimizer for the NFP nesting engine.
    ///
    /// Important:
    /// This optimizer does not directly move geometry. The geometry placement is still
    /// handled by NfpPlacementEngine.PlaceAll(order). This class searches for better
    /// part orders, because the current NFP engine is order-sensitive.
    ///
    /// Strategy:
    ///   1. Evaluate the incoming warm-start order.
    ///   2. Try several deterministic seed orders.
    ///   3. Run simulated annealing mutations until the time budget expires.
    ///   4. Score layouts by:
    ///        - unplaced part count first
    ///        - sheet count second
    ///        - packed envelope area third
    ///        - last-sheet usage fourth
    ///        - compactness / spread fifth
    ///
    /// Lower score is better.
    /// </summary>
    public static class SimulatedAnnealing
    {
        private const int MinIterationsBeforeCooling = 10;

        // Progress updates should not spam Rhino's command line / progress dialog.
        private const int ProgressUpdateMilliseconds = 500;

        // Safety: if the user enters a very tiny time budget, still run at least
        // the warm start and a couple of seed attempts.
        private static readonly TimeSpan MinimumBudget = TimeSpan.FromSeconds(1.0);

        /// <summary>
        /// Optimize the part order for the supplied NFP placement engine.
        ///
        /// This signature matches the existing call in NestingEngine.RunNfpInner.
        /// </summary>
        public static NfpPlacementEngine.NestResult Optimize(
            NfpPlacementEngine engine,
            IReadOnlyList<int> initialOrder,
            TimeSpan timeBudget,
            int randomSeed,
            Action<double, string> progressCallback)
        {
            if (engine == null) throw new ArgumentNullException(nameof(engine));
            if (initialOrder == null) throw new ArgumentNullException(nameof(initialOrder));
            if (initialOrder.Count == 0)
                return engine.PlaceAll(initialOrder);

            if (timeBudget < MinimumBudget)
                timeBudget = MinimumBudget;

            var random = new Random(randomSeed);
            var stopwatch = Stopwatch.StartNew();
            var lastProgress = Stopwatch.StartNew();

            // Phase 25.1: if user supplied a non-zero seed, randomize the
            // initial order via Fisher-Yates. This ensures the warm start,
            // the deterministic seed orders (which derive FROM the initial
            // order), and the annealing mutations all diverge by seed.
            // Seed 0 stays deterministic (largest-first) for production
            // reproducibility.
            var workingInitialOrder = new List<int>(initialOrder);
            if (randomSeed != 0)
            {
                for (int i = workingInitialOrder.Count - 1; i > 0; i--)
                {
                    int j = random.Next(i + 1);
                    int tmp = workingInitialOrder[i];
                    workingInitialOrder[i] = workingInitialOrder[j];
                    workingInitialOrder[j] = tmp;
                }
            }

            // ------------------------------------------------------------------
            // 1. Warm start
            // ------------------------------------------------------------------

            var currentOrder = new List<int>(workingInitialOrder);
            var currentResult = engine.PlaceAll(currentOrder);
            var currentScore = ScoreResult(currentResult);

            var bestOrder = new List<int>(currentOrder);
            var bestResult = currentResult;
            var bestScore = currentScore;

            progressCallback?.Invoke(
                0.0,
                FormatStatus("Annealing warm start", 0, bestScore, currentScore));

            // ------------------------------------------------------------------
            // 2. Deterministic seed orders
            // ------------------------------------------------------------------
            // These help escape the "largest-first only" trap before random annealing
            // begins. They are cheap and deterministic.
            // ------------------------------------------------------------------

            var seedOrders = BuildSeedOrders(workingInitialOrder);

            int iteration = 0;

            foreach (var seedOrder in seedOrders)
            {
                if (stopwatch.Elapsed >= timeBudget)
                    break;

                iteration++;

                var seedResult = engine.PlaceAll(seedOrder);
                var seedScore = ScoreResult(seedResult);

                if (IsBetter(seedScore, bestScore))
                {
                    bestScore = seedScore;
                    bestResult = seedResult;
                    bestOrder = new List<int>(seedOrder);
                }

                if (IsBetter(seedScore, currentScore))
                {
                    currentScore = seedScore;
                    currentResult = seedResult;
                    currentOrder = new List<int>(seedOrder);
                }

                MaybeReportProgress(
                    progressCallback,
                    stopwatch,
                    lastProgress,
                    timeBudget,
                    iteration,
                    bestScore,
                    currentScore);
            }

            // ------------------------------------------------------------------
            // 3. Simulated annealing loop
            // ------------------------------------------------------------------

            while (stopwatch.Elapsed < timeBudget)
            {
                iteration++;

                double progress = Clamp01(stopwatch.Elapsed.TotalSeconds / timeBudget.TotalSeconds);

                // Temperature starts high enough to accept meaningful worse moves,
                // then cools toward near-zero. The final 15% is mostly exploitation.
                double temperature = ComputeTemperature(progress, iteration);

                var trialOrder = new List<int>(currentOrder);
                MutateOrder(trialOrder, random, progress);

                var trialResult = engine.PlaceAll(trialOrder);
                var trialScore = ScoreResult(trialResult);

                bool betterThanCurrent = IsBetter(trialScore, currentScore);
                bool accept = betterThanCurrent ||
                              ShouldAcceptWorse(currentScore, trialScore, temperature, random);

                if (accept)
                {
                    currentOrder = trialOrder;
                    currentResult = trialResult;
                    currentScore = trialScore;
                }

                if (IsBetter(trialScore, bestScore))
                {
                    bestOrder = trialOrder;
                    bestResult = trialResult;
                    bestScore = trialScore;

                    // When a new best is found, continue from it. This makes the
                    // optimizer more aggressive for production nesting.
                    currentOrder = new List<int>(bestOrder);
                    currentResult = bestResult;
                    currentScore = bestScore;
                }

                // Occasional restart from the best solution prevents the search from
                // wandering too far after accepting several worse moves.
                if (iteration % 25 == 0)
                {
                    currentOrder = new List<int>(bestOrder);
                    currentResult = bestResult;
                    currentScore = bestScore;
                }

                // Occasional hard kick from the original order helps find a different
                // basin of solutions without losing the best result.
                if (iteration % 60 == 0)
                {
                    currentOrder = new List<int>(workingInitialOrder);
                    ApplyKickMutation(currentOrder, random);
                    currentResult = engine.PlaceAll(currentOrder);
                    currentScore = ScoreResult(currentResult);

                    if (IsBetter(currentScore, bestScore))
                    {
                        bestOrder = new List<int>(currentOrder);
                        bestResult = currentResult;
                        bestScore = currentScore;
                    }
                }

                MaybeReportProgress(
                    progressCallback,
                    stopwatch,
                    lastProgress,
                    timeBudget,
                    iteration,
                    bestScore,
                    currentScore);
            }

            progressCallback?.Invoke(
                1.0,
                FormatStatus("Annealing complete", iteration, bestScore, currentScore));

            return bestResult;
        }

        // ==================================================================
        // Seed order generation
        // ==================================================================

        private static List<List<int>> BuildSeedOrders(IReadOnlyList<int> initialOrder)
        {
            var result = new List<List<int>>();
            int n = initialOrder.Count;

            if (n <= 1)
                return result;

            // 1. Reverse order: sometimes smaller or shorter parts first create
            // fill opportunities that largest-first misses.
            var reversed = new List<int>(initialOrder);
            reversed.Reverse();
            result.Add(reversed);

            // 2. Front/back interleave: large, small, next large, next small.
            var interleaved = new List<int>(n);
            int left = 0;
            int right = n - 1;
            bool takeLeft = true;
            while (left <= right)
            {
                if (takeLeft)
                    interleaved.Add(initialOrder[left++]);
                else
                    interleaved.Add(initialOrder[right--]);

                takeLeft = !takeLeft;
            }
            result.Add(interleaved);

            // 3. Back/front interleave: small, large, next small, next large.
            var interleavedReverse = new List<int>(n);
            left = 0;
            right = n - 1;
            takeLeft = false;
            while (left <= right)
            {
                if (takeLeft)
                    interleavedReverse.Add(initialOrder[left++]);
                else
                    interleavedReverse.Add(initialOrder[right--]);

                takeLeft = !takeLeft;
            }
            result.Add(interleavedReverse);

            // 4. Rotate first third to the back.
            if (n >= 6)
            {
                int cut = Math.Max(1, n / 3);
                var rotated = new List<int>(n);
                for (int i = cut; i < n; i++) rotated.Add(initialOrder[i]);
                for (int i = 0; i < cut; i++) rotated.Add(initialOrder[i]);
                result.Add(rotated);
            }

            // 5. Rotate first half to the back.
            if (n >= 8)
            {
                int cut = n / 2;
                var rotated = new List<int>(n);
                for (int i = cut; i < n; i++) rotated.Add(initialOrder[i]);
                for (int i = 0; i < cut; i++) rotated.Add(initialOrder[i]);
                result.Add(rotated);
            }

            // 6. Alternating pairs: useful for long boat parts where a strict
            // size order causes diagonal sprawl.
            if (n >= 4)
            {
                var pairSwap = new List<int>(initialOrder);
                for (int i = 0; i + 1 < pairSwap.Count; i += 2)
                    Swap(pairSwap, i, i + 1);
                result.Add(pairSwap);
            }

            return RemoveDuplicateOrders(result);
        }

        private static List<List<int>> RemoveDuplicateOrders(List<List<int>> orders)
        {
            var result = new List<List<int>>();
            var seen = new HashSet<string>();

            foreach (var order in orders)
            {
                string key = string.Join(",", order);
                if (seen.Add(key))
                    result.Add(order);
            }

            return result;
        }

        // ==================================================================
        // Mutation operators
        // ==================================================================

        private static void MutateOrder(List<int> order, Random random, double progress)
        {
            int n = order.Count;
            if (n <= 1) return;

            // Early search: bigger disruptive moves.
            // Late search: smaller local refinements.
            int op;
            if (progress < 0.35)
                op = random.Next(0, 7);
            else if (progress < 0.75)
                op = random.Next(0, 6);
            else
                op = random.Next(0, 4);

            switch (op)
            {
                case 0:
                    MutateSwap(order, random);
                    break;

                case 1:
                    MutateMoveOne(order, random);
                    break;

                case 2:
                    MutateReverseWindow(order, random);
                    break;

                case 3:
                    MutateShuffleSmallWindow(order, random);
                    break;

                case 4:
                    MutateBlockMove(order, random);
                    break;

                case 5:
                    MutateMultipleSwaps(order, random, 2 + random.Next(3));
                    break;

                default:
                    ApplyKickMutation(order, random);
                    break;
            }
        }

        private static void ApplyKickMutation(List<int> order, Random random)
        {
            int n = order.Count;
            if (n <= 1) return;

            int moves = Math.Max(2, Math.Min(12, n / 4));

            for (int i = 0; i < moves; i++)
            {
                int op = random.Next(0, 5);
                switch (op)
                {
                    case 0:
                        MutateSwap(order, random);
                        break;
                    case 1:
                        MutateMoveOne(order, random);
                        break;
                    case 2:
                        MutateReverseWindow(order, random);
                        break;
                    case 3:
                        MutateBlockMove(order, random);
                        break;
                    default:
                        MutateShuffleSmallWindow(order, random);
                        break;
                }
            }
        }

        private static void MutateSwap(List<int> order, Random random)
        {
            int n = order.Count;
            if (n < 2) return;

            int a = random.Next(n);
            int b = random.Next(n);
            if (a == b) b = (b + 1) % n;

            Swap(order, a, b);
        }

        private static void MutateMultipleSwaps(List<int> order, Random random, int count)
        {
            for (int i = 0; i < count; i++)
                MutateSwap(order, random);
        }

        private static void MutateMoveOne(List<int> order, Random random)
        {
            int n = order.Count;
            if (n < 2) return;

            int from = random.Next(n);
            int to = random.Next(n);

            if (from == to) return;

            int value = order[from];
            order.RemoveAt(from);

            if (to > from) to--;
            if (to < 0) to = 0;
            if (to > order.Count) to = order.Count;

            order.Insert(to, value);
        }

        private static void MutateReverseWindow(List<int> order, Random random)
        {
            int n = order.Count;
            if (n < 4)
            {
                MutateSwap(order, random);
                return;
            }

            int maxLen = Math.Max(2, Math.Min(n, n / 3));
            int len = 2 + random.Next(maxLen - 1);
            int start = random.Next(0, n - len + 1);

            order.Reverse(start, len);
        }

        private static void MutateShuffleSmallWindow(List<int> order, Random random)
        {
            int n = order.Count;
            if (n < 4)
            {
                MutateSwap(order, random);
                return;
            }

            int maxLen = Math.Max(3, Math.Min(n, 8));
            int len = 3 + random.Next(maxLen - 2);
            int start = random.Next(0, n - len + 1);

            for (int i = start + len - 1; i > start; i--)
            {
                int j = random.Next(start, i + 1);
                Swap(order, i, j);
            }
        }

        private static void MutateBlockMove(List<int> order, Random random)
        {
            int n = order.Count;
            if (n < 5)
            {
                MutateMoveOne(order, random);
                return;
            }

            int maxBlock = Math.Max(2, Math.Min(n / 3, 10));
            int blockLen = 2 + random.Next(maxBlock - 1);
            int from = random.Next(0, n - blockLen + 1);

            var block = order.GetRange(from, blockLen);
            order.RemoveRange(from, blockLen);

            int to = random.Next(0, order.Count + 1);
            order.InsertRange(to, block);
        }

        private static void Swap(List<int> order, int a, int b)
        {
            if (a == b) return;

            int temp = order[a];
            order[a] = order[b];
            order[b] = temp;
        }

        // ==================================================================
        // Scoring
        // ==================================================================

        private static SolutionScore ScoreResult(NfpPlacementEngine.NestResult result)
        {
            if (result == null)
            {
                return new SolutionScore
                {
                    UnplacedCount = int.MaxValue,
                    SheetCount = int.MaxValue,
                    TotalEnvelopeArea = double.MaxValue,
                    WorstSheetEnvelopeArea = double.MaxValue,
                    TotalUsedRight = double.MaxValue,
                    TotalUsedTop = double.MaxValue,
                    SpreadPenalty = double.MaxValue
                };
            }

            var sheetStats = new Dictionary<int, SheetScoreStats>();

            foreach (var placement in result.Placements)
            {
                if (!sheetStats.TryGetValue(placement.Sheet, out var stats))
                {
                    stats = new SheetScoreStats
                    {
                        MinX = double.MaxValue,
                        MinY = double.MaxValue,
                        MaxX = double.MinValue,
                        MaxY = double.MinValue,
                        PartCount = 0,
                        PartArea = 0.0
                    };
                }

                var bb = placement.PlacedPolygon.BoundingBox;

                stats.MinX = Math.Min(stats.MinX, bb.MinX);
                stats.MinY = Math.Min(stats.MinY, bb.MinY);
                stats.MaxX = Math.Max(stats.MaxX, bb.MaxX);
                stats.MaxY = Math.Max(stats.MaxY, bb.MaxY);
                stats.PartCount++;
                stats.PartArea += placement.PlacedPolygon.AbsoluteArea;

                sheetStats[placement.Sheet] = stats;
            }

            double totalEnvelopeArea = 0.0;
            double worstEnvelopeArea = 0.0;
            double totalUsedRight = 0.0;
            double totalUsedTop = 0.0;
            double spreadPenalty = 0.0;

            foreach (var kv in sheetStats)
            {
                var stats = kv.Value;

                double width = Math.Max(0.0, stats.MaxX - Math.Min(0.0, stats.MinX));
                double height = Math.Max(0.0, stats.MaxY - Math.Min(0.0, stats.MinY));

                double envelopeArea = width * height;

                totalEnvelopeArea += envelopeArea;
                worstEnvelopeArea = Math.Max(worstEnvelopeArea, envelopeArea);
                totalUsedRight += stats.MaxX;
                totalUsedTop += stats.MaxY;

                // Empty space inside the occupied envelope. Smaller is better.
                spreadPenalty += Math.Max(0.0, envelopeArea - stats.PartArea);

                // Slight penalty for very few parts on a sheet, especially the last
                // sheet. This helps pull parts back from weak overflow sheets when
                // another order can make them fit earlier.
                if (stats.PartCount <= 2)
                    spreadPenalty += envelopeArea * 0.05;
            }

            int actualSheetCount = Math.Max(result.SheetCount, sheetStats.Count);

            return new SolutionScore
            {
                UnplacedCount = result.Unplaced == null ? 0 : result.Unplaced.Count,
                SheetCount = actualSheetCount,
                TotalEnvelopeArea = totalEnvelopeArea,
                WorstSheetEnvelopeArea = worstEnvelopeArea,
                TotalUsedRight = totalUsedRight,
                TotalUsedTop = totalUsedTop,
                SpreadPenalty = spreadPenalty
            };
        }

        private static bool IsBetter(SolutionScore a, SolutionScore b)
        {
            if (a.UnplacedCount != b.UnplacedCount)
                return a.UnplacedCount < b.UnplacedCount;

            if (a.SheetCount != b.SheetCount)
                return a.SheetCount < b.SheetCount;

            if (Different(a.TotalEnvelopeArea, b.TotalEnvelopeArea))
                return a.TotalEnvelopeArea < b.TotalEnvelopeArea;

            if (Different(a.WorstSheetEnvelopeArea, b.WorstSheetEnvelopeArea))
                return a.WorstSheetEnvelopeArea < b.WorstSheetEnvelopeArea;

            if (Different(a.SpreadPenalty, b.SpreadPenalty))
                return a.SpreadPenalty < b.SpreadPenalty;

            if (Different(a.TotalUsedTop, b.TotalUsedTop))
                return a.TotalUsedTop < b.TotalUsedTop;

            if (Different(a.TotalUsedRight, b.TotalUsedRight))
                return a.TotalUsedRight < b.TotalUsedRight;

            return false;
        }

        private static bool Different(double a, double b)
        {
            return Math.Abs(a - b) > 1e-7;
        }

        /// <summary>
        /// Scalar energy for simulated annealing acceptance.
        ///
        /// The lexicographic IsBetter method is used for "best" decisions.
        /// This scalar is only for probabilistic acceptance of worse moves.
        /// </summary>
        private static double Energy(SolutionScore s)
        {
            double energy = 0.0;

            // These weights are intentionally huge. A layout with fewer unplaced
            // parts or fewer sheets should dominate small compactness improvements.
            energy += s.UnplacedCount * 1.0e15;
            energy += s.SheetCount * 1.0e12;

            energy += s.TotalEnvelopeArea * 1.0e4;
            energy += s.WorstSheetEnvelopeArea * 1.0e3;
            energy += s.SpreadPenalty * 250.0;
            energy += s.TotalUsedTop * 50.0;
            energy += s.TotalUsedRight * 10.0;

            return energy;
        }

        private static bool ShouldAcceptWorse(
            SolutionScore current,
            SolutionScore trial,
            double temperature,
            Random random)
        {
            if (temperature <= 1e-12)
                return false;

            double currentEnergy = Energy(current);
            double trialEnergy = Energy(trial);
            double delta = trialEnergy - currentEnergy;

            if (delta <= 0.0)
                return true;

            // Normalize delta so large absolute nesting numbers do not make
            // acceptance impossible.
            double scale = Math.Max(1.0, Math.Abs(currentEnergy));
            double normalizedDelta = delta / scale;

            double probability = Math.Exp(-normalizedDelta / temperature);

            return random.NextDouble() < probability;
        }

        private static double ComputeTemperature(double progress, int iteration)
        {
            progress = Clamp01(progress);

            if (iteration < MinIterationsBeforeCooling)
                return 0.08;

            // Smooth cooling curve.
            double t = 1.0 - progress;
            double temperature = 0.0005 + 0.08 * t * t;

            return temperature;
        }

        // ==================================================================
        // Progress / formatting
        // ==================================================================

        private static void MaybeReportProgress(
            Action<double, string> progressCallback,
            Stopwatch stopwatch,
            Stopwatch lastProgress,
            TimeSpan timeBudget,
            int iteration,
            SolutionScore bestScore,
            SolutionScore currentScore)
        {
            if (progressCallback == null)
                return;

            if (lastProgress.ElapsedMilliseconds < ProgressUpdateMilliseconds)
                return;

            lastProgress.Restart();

            double progress = Clamp01(stopwatch.Elapsed.TotalSeconds / timeBudget.TotalSeconds);

            progressCallback.Invoke(
                progress,
                FormatStatus("Annealing", iteration, bestScore, currentScore));
        }

        private static string FormatStatus(
            string prefix,
            int iteration,
            SolutionScore best,
            SolutionScore current)
        {
            return
                $"{prefix}: iter {iteration}, " +
                $"best {best.SheetCount} sheet(s), {best.UnplacedCount} unplaced, " +
                $"current {current.SheetCount} sheet(s), {current.UnplacedCount} unplaced";
        }

        private static double Clamp01(double value)
        {
            if (value < 0.0) return 0.0;
            if (value > 1.0) return 1.0;
            return value;
        }

        // ==================================================================
        // Internal score structs
        // ==================================================================

        private struct SolutionScore
        {
            public int UnplacedCount;
            public int SheetCount;
            public double TotalEnvelopeArea;
            public double WorstSheetEnvelopeArea;
            public double TotalUsedRight;
            public double TotalUsedTop;
            public double SpreadPenalty;
        }

        private struct SheetScoreStats
        {
            public double MinX;
            public double MinY;
            public double MaxX;
            public double MaxY;
            public int PartCount;
            public double PartArea;
        }
    }
}
