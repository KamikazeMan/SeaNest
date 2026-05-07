using System.Collections.Concurrent;
using System.Collections.Generic;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// Lazy, thread-safe cache of pairwise No-Fit Polygons keyed by orientation index.
    ///
    /// The placement engine asks for the NFP of a placed part B against a candidate
    /// part A many times across many simulated-annealing iterations. Computing those
    /// NFPs is the dominant cost of NFP-based nesting; caching them turns annealing
    /// from impractical into routine.
    ///
    /// Cache key: ordered pair (orientationIndex_A, orientationIndex_B). NFPs are NOT
    /// symmetric — NFP(A, B) ≠ NFP(B, A) in general — so the pair is ordered.
    ///
    /// Lazy population: entries compute on first access and stick around for the rest
    /// of the nest. For a typical 30-part job with mirror and Every15 rotation, the
    /// orientation count is ~1440 and the upper-bound cache size is ~2M entries, but
    /// the working set during annealing is usually 5–15% of that. Lazy is cheaper than
    /// precompute-all and produces identical placement results.
    ///
    /// Spacing handling: the request's <see cref="NestRequest.Spacing"/> is baked into
    /// the cached NFP (passed through to <see cref="NoFitPolygon.Compute"/>). The cache
    /// is therefore valid for one nest only — calling code must construct a fresh
    /// <see cref="NfpCache"/> per nest and discard it after. Re-using a cache across
    /// nests with different spacing requirements would silently produce wrong results.
    /// </summary>
    public sealed class NfpCache
    {
        private readonly ConcurrentDictionary<long, IReadOnlyList<Polygon>> _entries
            = new ConcurrentDictionary<long, IReadOnlyList<Polygon>>();

        private readonly double _spacing;

        /// <summary>
        /// Construct a cache for one nest. <paramref name="spacing"/> is baked into
        /// every NFP computed through this cache and must match the
        /// <see cref="NestRequest.Spacing"/> of the active request.
        /// </summary>
        public NfpCache(double spacing)
        {
            _spacing = spacing;
        }

        /// <summary>
        /// Number of cache entries currently populated. Diagnostic only.
        /// </summary>
        public int Count => _entries.Count;

        /// <summary>
        /// Get the NFP of <paramref name="b"/> with respect to <paramref name="a"/>,
        /// computing and caching it on first access.
        ///
        /// Returns the polygon list as <see cref="NoFitPolygon.Compute"/> produced it.
        /// Multiple polygons can occur for thin or L-shaped A; an empty list indicates
        /// degenerate inputs and the placement engine should treat it as "any
        /// translation of B avoids A" (no forbidden region from this pair).
        /// </summary>
        public IReadOnlyList<Polygon> Get(OrientedPart a, OrientedPart b)
        {
            long key = MakeKey(a.OrientationIndex, b.OrientationIndex);

            // Fast path: already in cache.
            if (_entries.TryGetValue(key, out var cached))
                return cached;

            // Cold path: compute and add. ConcurrentDictionary handles the race —
            // if two threads compute the same key concurrently, both compute (waste),
            // one wins the GetOrAdd, the other's result is discarded. NoFitPolygon.Compute
            // is pure (no side effects), so the duplicate work is harmless and avoids
            // the more expensive lock-on-miss pattern.
            var fresh = NoFitPolygon.Compute(
                a.CanonicalPolygon, b.CanonicalPolygon, _spacing,
                a.OrientationIndex, b.OrientationIndex);
            return _entries.GetOrAdd(key, fresh);
        }

        /// <summary>
        /// Drop all cached entries. The cache is normally discarded along with the
        /// engine; this method exists for diagnostics and tests.
        /// </summary>
        public void Clear() => _entries.Clear();

        /// <summary>
        /// Pack two int32 orientation indices into one int64 cache key. Direction-
        /// preserving (a,b distinct from b,a) and collision-free for any reasonable
        /// orientation count (up to 2^31).
        /// </summary>
        private static long MakeKey(int orientationA, int orientationB)
            => ((long)(uint)orientationA << 32) | (uint)orientationB;
    }
}