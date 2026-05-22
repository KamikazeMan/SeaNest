using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Lazy, thread-safe, single-flight cache of pairwise No-Fit Polygons.
    ///
    /// Phase 22b:
    /// Uses Lazy&lt;IReadOnlyList&lt;Polygon&gt;&gt; inside ConcurrentDictionary so multiple
    /// threads requesting the same pair wait on one computation instead of all
    /// computing the same expensive MinkowskiDiff.
    /// </summary>
    public sealed class NfpCache
    {
        private readonly ConcurrentDictionary<long, Lazy<IReadOnlyList<Polygon>>> _entries
            = new ConcurrentDictionary<long, Lazy<IReadOnlyList<Polygon>>>();

        private readonly double _spacing;

        public NfpCache(double spacing)
        {
            _spacing = spacing;
        }

        public int Count => _entries.Count;

        public IReadOnlyList<Polygon> Get(OrientedPart a, OrientedPart b)
        {
            if (a == null) throw new ArgumentNullException(nameof(a));
            if (b == null) throw new ArgumentNullException(nameof(b));

            long key = MakeKey(a.OrientationIndex, b.OrientationIndex);

            var lazy = _entries.GetOrAdd(
                key,
                _ => new Lazy<IReadOnlyList<Polygon>>(
                    () => NoFitPolygon.Compute(
                        a.CanonicalPolygon,
                        b.CanonicalPolygon,
                        _spacing,
                        a.OrientationIndex,
                        b.OrientationIndex),
                    LazyThreadSafetyMode.ExecutionAndPublication));

            try
            {
                return lazy.Value;
            }
            catch
            {
                Lazy<IReadOnlyList<Polygon>> ignored;
                _entries.TryRemove(key, out ignored);
                throw;
            }
        }

        public void Clear() => _entries.Clear();

        private static long MakeKey(int orientationA, int orientationB)
            => ((long)(uint)orientationA << 32) | (uint)orientationB;
    }
}
