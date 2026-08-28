using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Threading.Tasks;
using Clipper2Lib;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.Nesting.Core.Overlap;

namespace SeaNest.Nesting.Core.Nesting
{
    /// <summary>
    /// NFP-based placement engine.
    ///
    /// Phase Smart-Placement upgrade:
    /// Instead of taking only the single bottom-left vertex from the feasible
    /// region, this engine now collects multiple candidate vertices from the
    /// feasible region and scores them for compactness, used sheet area, used
    /// sheet height, used sheet length, and contact with sheet/part edges.
    ///
    /// This keeps the existing NFP/IFP/Clipper architecture but makes the local
    /// placement decision smarter.
    /// </summary>
    public sealed class NfpPlacementEngine
    {
        private readonly NestRequest _request;
        private readonly IReadOnlyList<List<OrientedPart>> _orientationsByPart;
        private readonly NfpCache _nfpCache;

        // Phase 28.3.1: effective regret weight (see DefaultRegretWeight),
        // resolved once from SEANEST_REGRET_WEIGHT at construction.
        private readonly double _regretWeight;

        // Phase 30 increment 3: frame-nester weights resolved once at
        // construction from env vars (see Default* constants below).
        private readonly double _frameContactWeight;
        private readonly double _frameBboxWeight;
        private readonly double _contactInflateMultiplier;

        public Action<string> DiagnosticLog { get; set; }

        public bool EnableInteriorSampling { get; set; } = false;
        public double? InteriorSamplingStep { get; set; } = null;

        public TimeSpan? BeamRetryTimeBudget { get; set; }
        public int BeamWidth { get; set; } = 32;
        public int PlacementsPerPart { get; set; } = 4;
        public int BeamMaxCandidates { get; set; } = 256;

        public IReadOnlyList<int> CriticalPartIndices { get; set; }
        public HashSet<int> IrregularPartIndices { get; set; }

        private static readonly string InteriorFallbackLogPath =
            Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory),
                "phase26_interior.txt");

        private static readonly List<string> InteriorFallbackLog = new List<string>();

        public NfpPlacementEngine(
            NestRequest request,
            IReadOnlyList<List<OrientedPart>> orientationsByPart,
            NfpCache nfpCache)
        {
            _request = request ?? throw new ArgumentNullException(nameof(request));
            _orientationsByPart = orientationsByPart ?? throw new ArgumentNullException(nameof(orientationsByPart));
            _nfpCache = nfpCache ?? throw new ArgumentNullException(nameof(nfpCache));

            double EnvDouble(string name, double fallback) =>
                double.TryParse(
                    Environment.GetEnvironmentVariable(name),
                    System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture,
                    out double parsed)
                    ? parsed
                    : fallback;

            _regretWeight = EnvDouble("SEANEST_REGRET_WEIGHT", DefaultRegretWeight);
            _frameContactWeight = EnvDouble("SEANEST_FRAME_CONTACT_W", DefaultFrameContactWeight);
            _frameBboxWeight = EnvDouble("SEANEST_FRAME_BBOX_W", DefaultFrameBboxWeight);
            _contactInflateMultiplier = EnvDouble("SEANEST_FRAME_INFLATE_MULT", DefaultContactInflateMultiplier);
        }

        public NestResult PlaceAll(IReadOnlyList<int> partOrder)
        {
            if (partOrder == null) throw new ArgumentNullException(nameof(partOrder));

            InteriorFallbackLog.Clear();

            var sheets = new List<SheetState>();
            var placements = new List<PlacementResult>();
            var unplaced = new List<int>();

            foreach (int partIndex in partOrder)
            {
                var orientations = _orientationsByPart[partIndex];

                bool placed = false;

                for (int sheetIdx = 0; sheetIdx < sheets.Count && !placed; sheetIdx++)
                {
                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        sheets[sheetIdx],
                        sheetIdx,
                        placements);
                }

                if (!placed)
                {
                    var newSheet = new SheetState();
                    sheets.Add(newSheet);

                    int newSheetIdx = sheets.Count - 1;

                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        newSheet,
                        newSheetIdx,
                        placements);

                    if (!placed)
                    {
                        sheets.RemoveAt(newSheetIdx);
                        unplaced.Add(partIndex);
                    }
                }
            }

            FlushInteriorFallbackLog();

            return new NestResult(sheets.Count, placements, unplaced);
        }

        public readonly struct PreplacedPart
        {
            public int OriginalIndex { get; }
            public int Sheet { get; }
            public OrientedPart Orientation { get; }
            public double X { get; }
            public double Y { get; }
            public PlacementResult Placement { get; }

            public PreplacedPart(
                int originalIndex, int sheet, OrientedPart orientation,
                double x, double y, PlacementResult placement)
            {
                OriginalIndex = originalIndex;
                Sheet = sheet;
                Orientation = orientation;
                X = x;
                Y = y;
                Placement = placement;
            }
        }

        public NestResult PlaceAllWithPreplaced(
            IReadOnlyList<int> partOrder,
            IReadOnlyList<PreplacedPart> preplaced)
        {
            if (partOrder == null) throw new ArgumentNullException(nameof(partOrder));
            if (preplaced == null) throw new ArgumentNullException(nameof(preplaced));

            var sheets = new List<SheetState>();
            var placements = new List<PlacementResult>();
            var unplaced = new List<int>();

            // Seed sheet state from pre-placed parts.
            foreach (var pp in preplaced)
            {
                while (sheets.Count <= pp.Sheet)
                    sheets.Add(new SheetState());

                sheets[pp.Sheet].Placed.Add(
                    new PlacedItem(pp.Orientation, pp.X, pp.Y));

                placements.Add(pp.Placement);
            }

            // Clear forbidden-region caches on all seeded sheets so the first
            // real placement rebuilds them against the full pre-placed state.
            foreach (var sheet in sheets)
                sheet.ForbiddenByOrientation.Clear();

            // Now place the new parts normally.
            foreach (int partIndex in partOrder)
            {
                var orientations = _orientationsByPart[partIndex];

                bool placed = false;

                for (int sheetIdx = 0; sheetIdx < sheets.Count && !placed; sheetIdx++)
                {
                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        sheets[sheetIdx],
                        sheetIdx,
                        placements);
                }

                if (!placed)
                {
                    var newSheet = new SheetState();
                    sheets.Add(newSheet);

                    int newSheetIdx = sheets.Count - 1;

                    placed = TryPlaceOnSheet(
                        partIndex,
                        orientations,
                        newSheet,
                        newSheetIdx,
                        placements);

                    if (!placed)
                    {
                        sheets.RemoveAt(newSheetIdx);
                        unplaced.Add(partIndex);
                    }
                }
            }

            return new NestResult(sheets.Count, placements, unplaced);
        }

        // Phase 30 (increment 1): coordinated frame nester. Greedy best-fit
        // placement of the large hull frames at FIXED orientation (each frame's
        // orientation index 0 — no rotation search yet), then hand the placed
        // frames to PlaceAllWithPreplaced so the existing engine fills the rest.
        //
        // frameIndices : the parts to coordinate (CriticalPartIndices).
        // remainingOrder: every other part, in the order the fill should attempt.
        // diag         : diagnostic lines appended for the caller to persist.
        //
        // Placeholder fit score (increment 1): bottom-left bias (minimize Y,
        // then X). The real profile-contact metric is increment 3. Frames that
        // cannot be placed against the stack are appended to the fill order so
        // PlaceAllWithPreplaced can place them or spill them — never overlap,
        // never hard-fail. Deterministic: frames seeded largest-area-first
        // (index tiebreak), candidates from the deterministic FindCandidateVertices,
        // ties broken by (Y, X, frameIndex). No parallelism.
        public NestResult PlaceCoordinatedFrames(
            IReadOnlyList<int> frameIndices,
            IReadOnlyList<int> remainingOrder,
            List<string> diag)
        {
            if (frameIndices == null) throw new ArgumentNullException(nameof(frameIndices));
            if (remainingOrder == null) throw new ArgumentNullException(nameof(remainingOrder));
            if (diag == null) diag = new List<string>();

            // Fixed orientation = each frame's orientation index 0 (rot 0, no mirror).
            OrientedPart Orient0(int part) => _orientationsByPart[part][0];

            // A scratch sheet whose Placed list holds the committed frames; used
            // to build the union forbidden region via the existing helper.
            var sheet = new SheetState();
            var committed = new List<(int Part, OrientedPart Orient, double X, double Y)>();

            // Pending frames, seeded largest-area first (index tiebreak).
            var pending = new List<int>(frameIndices);
            pending.Sort((a, b) =>
            {
                int cmp = Orient0(b).CanonicalPolygon.AbsoluteArea
                    .CompareTo(Orient0(a).CanonicalPolygon.AbsoluteArea);
                if (cmp != 0) return cmp;
                return a.CompareTo(b);
            });

            int totalFrames = pending.Count;
            diag.Add($"Frames to place ({totalFrames}), largest-first: " +
                     string.Join(",", pending));

            // Phase 30 increment 3 state.
            double inflateDelta = _contactInflateMultiplier * _request.Spacing;
            diag.Add($"Contact metric: w_bbox={_frameBboxWeight:F2}, " +
                     $"w_contact={_frameContactWeight:F2}, " +
                     $"inflate delta={inflateDelta:F3} ({_contactInflateMultiplier:F2}x spacing) " +
                     "[env-tunable: SEANEST_FRAME_BBOX_W / _CONTACT_W / _INFLATE_MULT].");

            // ----------------------------------------------------------------
            // Phase 30 (speed, dual geometry): frame-phase measured at 1698s of
            // a 2025s catamaran run — 84% of total — because every NFP,
            // candidate sweep, and contact Intersect ran on raw ~200-450 vertex
            // concave frame polygons. Fix: score with SIMPLIFIED proxies, commit
            // with FULL-RES geometry.
            //
            //   Proxy side  : NFP computation, candidate generation, overlap
            //                 pre-check, contact-area scoring.
            //   Full-res side: IFP (on-sheet bound), bbox-growth term, the
            //                 commit-time TRUE-polygon overlap gate, the
            //                 emitted PreplacedPart (output + fill + holes).
            //
            // Proxies come from NestingEngine.BuildNfpProxyStatic — the exact
            // simplification method, tolerances (0.01" start, 1.5x escalation,
            // 0.05" cap), drift guards (0.5% area, 0.05" bbox), and 80-vertex
            // target the main NFP path already uses. Deterministic: fixed
            // tolerance ladder, same input -> same proxy.
            //
            // Proxy orientations share the full-res OrientationIndex so the
            // proxy NfpCache keys stay aligned with the full-res set.
            // ----------------------------------------------------------------
            var proxyOrientByIndex = new Dictionary<int, OrientedPart>();
            foreach (int f in pending)
            {
                var rawSource = _request.Polygons[f];
                var proxySource = NestingEngine.BuildNfpProxyStatic(rawSource, f, DiagnosticLog);
                diag.Add($"  Frame {f} NFP proxy: {rawSource.Count} -> {proxySource.Count} verts.");

                foreach (var fo in _orientationsByPart[f])
                {
                    proxyOrientByIndex[fo.OrientationIndex] = OrientedPart.Build(
                        fo.OrientationIndex, f, proxySource, fo.RotationDeg, fo.IsMirrored);
                }
            }

            // Frame-phase-local proxy NFP cache. Same spacing as the main cache;
            // never mixed with _nfpCache (which stays full-res for the fill).
            var proxyCache = new NfpCache(_request.Spacing);

            // Forbidden region for a candidate proxy orientation against the
            // committed proxy placements. Mirrors GetOrBuildForbiddenRegion but
            // reads the proxy cache. Cached per orientation index in the scratch
            // sheet; invalidated at commit.
            Paths64 BuildFrameForbidden(OrientedPart candProxy)
            {
                if (sheet.Placed.Count == 0)
                    return new Paths64();
                if (sheet.ForbiddenByOrientation.TryGetValue(
                        candProxy.OrientationIndex, out var cached))
                    return cached;

                var allNfps = new Paths64();
                foreach (var placed in sheet.Placed)
                {
                    var nfp = proxyCache.Get(placed.Orientation, candProxy);
                    foreach (var nfpPoly in nfp)
                        allNfps.Add(ClipperConvert.ToPath64(
                            nfpPoly.Translate(placed.X, placed.Y)));
                }

                var union = allNfps.Count == 0
                    ? new Paths64()
                    : Clipper.Union(allNfps, FillRule.NonZero);
                sheet.ForbiddenByOrientation[candProxy.OrientationIndex] = union;
                return union;
            }

            // Inflated PROXY-canonical paths cached per orientation (translation
            // just shifts Path64 points; the dilated shape itself doesn't change).
            var inflatedCanonByOrient = new Dictionary<int, Paths64>();

            // Union of placed-frame PROXY polygons (Paths64), updated on each
            // commit. Contact-area scoring only; the true-polygon commit gate
            // never touches this.
            Paths64 placedFramesUnion = new Paths64();

            // Combined bbox of all placed frames (FULL-Res extents), tracked
            // incrementally.
            bool hasCommitted = false;
            double curMinX = 0, curMinY = 0, curMaxX = 0, curMaxY = 0;
            double curHalfPerim = 0.0;

            // True-polygon commit-gate rejections across the whole frame phase
            // (candidates that scored valid on proxies but overlapped on
            // full-res geometry — expected to be rare; surfaced for diagnosis).
            int trueGateRejects = 0;

            // Phase 30 (speed): explicit frame-phase / fill-phase timing.
            // The fill (PlaceAllWithPreplaced) runs exactly once at the end of
            // this method — this stopwatch proves it and shows the phase split.
            var framePhaseSw = System.Diagnostics.Stopwatch.StartNew();

            // Greedy loop: each iteration commits the single best valid (frame,
            // orientation, position) across all pending frames by the contact-
            // metric score (lower = better).
            while (pending.Count > 0)
            {
                DiagnosticLog?.Invoke(
                    $"Coordinated frames: placing frame {committed.Count + 1} of {totalFrames}...");

                // Candidate collection for this greedy iteration. All scoring
                // below runs on PROXY geometry; the sorted walk after the scan
                // applies the TRUE-polygon commit gate and takes the best
                // candidate that survives it ("simplified-valid but
                // true-invalid -> reject, try next candidate").
                var iterationCandidates =
                    new List<(double Score, int OrientIdx, double Y, double X,
                              int Frame, OrientedPart FullOrient,
                              double Contact, double BboxGrow)>();

                // Diagnosis: highest contact area achievable by any VALID
                // (non-overlapping) candidate per pending frame this step,
                // independent of score. If this is large but the committed
                // contact is tiny, the bbox term is suppressing high-contact
                // positions (weight problem); if this is also tiny, no position
                // achieves real cradling (contact-formulation/geometry problem).
                var bestAvailContact = new Dictionary<int, double>();

                foreach (int f in pending)
                {
                    // Try ALL orientations (ascending OrientationIndex).
                    foreach (var fo in _orientationsByPart[f])
                    {
                        // IFP from the FULL-RES orientation: the proxy bbox can
                        // be up to the drift guard (0.05") smaller, and an IFP
                        // computed from it could let the true polygon cross the
                        // sheet margin. Full-res IFP keeps the on-sheet bound
                        // exact.
                        var ifp = InnerFitPolygon.Compute(
                            fo, _request.SheetWidth, _request.SheetHeight, _request.Margin);
                        if (!ifp.HasValue) continue;

                        var foProxy = proxyOrientByIndex[fo.OrientationIndex];

                        // Forbidden region from PROXY NFPs against committed
                        // PROXY placements. Cached per orientation index; the
                        // cache is valid across the whole scoring pass because
                        // sheet.Placed only changes at commit (which clears it).
                        Paths64 forbidden = BuildFrameForbidden(foProxy);
                        Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);
                        if (feasible.Count == 0) continue;

                        // Cache the inflated PROXY-canonical Paths64 once per
                        // orientation — the dilated shape is translation-
                        // invariant, so we just shift the cached Path64 points
                        // per candidate.
                        if (!inflatedCanonByOrient.TryGetValue(
                                fo.OrientationIndex, out var infCanonPaths))
                        {
                            var inflated = PolygonInflate.Inflate(
                                foProxy.CanonicalPolygon, inflateDelta);
                            infCanonPaths = new Paths64();
                            foreach (var p in inflated)
                                infCanonPaths.Add(ClipperConvert.ToPath64(p));
                            inflatedCanonByOrient[fo.OrientationIndex] = infCanonPaths;
                        }

                        var candidates = FindCandidateVertices(feasible, MaxCandidateVertices);
                        foreach (var cand in candidates)
                        {
                            // Overlap PRE-check on PROXY polygons (cheap). The
                            // authoritative gate is the TRUE-polygon check in
                            // the sorted commit walk below.
                            var proxyPoly = foProxy.CanonicalPolygon.Translate(cand.X, cand.Y);
                            bool overlaps = false;
                            foreach (var c in committed)
                            {
                                var priorProxy = proxyOrientByIndex[c.Orient.OrientationIndex]
                                    .CanonicalPolygon.Translate(c.X, c.Y);
                                if (OverlapChecker.Overlaps(proxyPoly, priorProxy, OverlapTolerance))
                                {
                                    overlaps = true;
                                    break;
                                }
                            }
                            if (overlaps) continue;

                            // Bbox growth: combined half-perimeter delta, from
                            // FULL-RES orientation extents (the true footprint).
                            double cMaxX = cand.X + fo.BBox.MaxX;
                            double cMaxY = cand.Y + fo.BBox.MaxY;
                            double newMinX = hasCommitted ? Math.Min(curMinX, cand.X) : cand.X;
                            double newMinY = hasCommitted ? Math.Min(curMinY, cand.Y) : cand.Y;
                            double newMaxX = hasCommitted ? Math.Max(curMaxX, cMaxX) : cMaxX;
                            double newMaxY = hasCommitted ? Math.Max(curMaxY, cMaxY) : cMaxY;
                            double bboxGrow = (newMaxX - newMinX) + (newMaxY - newMinY) - curHalfPerim;

                            // Contact area: translated inflated PROXY candidate
                            // ∩ placed PROXY union. Score-only; correctness is
                            // guarded by the true-polygon commit gate.
                            double contactArea = 0.0;
                            if (placedFramesUnion.Count > 0)
                            {
                                long tx = (long)(cand.X * ClipperConvert.Scale);
                                long ty = (long)(cand.Y * ClipperConvert.Scale);
                                var translated = new Paths64(infCanonPaths.Count);
                                foreach (var path in infCanonPaths)
                                {
                                    var np = new Path64(path.Count);
                                    foreach (var pt in path)
                                        np.Add(new Point64(pt.X + tx, pt.Y + ty));
                                    translated.Add(np);
                                }
                                var inter = Clipper.Intersect(
                                    translated, placedFramesUnion, FillRule.NonZero);
                                if (inter != null)
                                {
                                    double scaled = 0.0;
                                    foreach (var path in inter)
                                    {
                                        double a = Clipper.Area(path);
                                        if (a < 0) a = -a;
                                        scaled += a;
                                    }
                                    contactArea = scaled /
                                        (ClipperConvert.Scale * ClipperConvert.Scale);
                                }
                            }

                            // Track the best contact achievable by any valid
                            // candidate for this frame (score-independent).
                            if (!bestAvailContact.TryGetValue(f, out double prevAvail)
                                || contactArea > prevAvail)
                                bestAvailContact[f] = contactArea;

                            double score = _frameBboxWeight * bboxGrow
                                         - _frameContactWeight * contactArea;

                            iterationCandidates.Add(
                                (score, fo.OrientationIndex, cand.Y, cand.X,
                                 f, fo, contactArea, bboxGrow));
                        }
                    }
                }

                // Deterministic total order (Phase 30 increment 3 semantics):
                // lower score wins; ties by orientation index, Y, X, frame
                // index — so repeated runs are byte-identical regardless of
                // enumeration order.
                iterationCandidates.Sort((a, b) =>
                {
                    int cmp = a.Score.CompareTo(b.Score);
                    if (cmp != 0) return cmp;
                    cmp = a.OrientIdx.CompareTo(b.OrientIdx);
                    if (cmp != 0) return cmp;
                    cmp = a.Y.CompareTo(b.Y);
                    if (cmp != 0) return cmp;
                    cmp = a.X.CompareTo(b.X);
                    if (cmp != 0) return cmp;
                    return a.Frame.CompareTo(b.Frame);
                });

                // Commit walk: take the best-scored candidate that passes the
                // TRUE full-resolution overlap gate. Proxy-valid-but-true-
                // invalid candidates are skipped (counted for diagnosis), NOT
                // spilled — the next candidate gets its chance.
                int bestFrame = -1;
                double bestScore = 0, bestY = 0, bestX = 0;
                OrientedPart bestOrient = null;
                double bestContact = 0.0, bestBboxGrow = 0.0;

                foreach (var candEntry in iterationCandidates)
                {
                    var truePoly = candEntry.FullOrient.CanonicalPolygon
                        .Translate(candEntry.X, candEntry.Y);
                    bool trueOverlap = false;
                    foreach (var c in committed)
                    {
                        if (OverlapChecker.Overlaps(
                                truePoly,
                                c.Orient.CanonicalPolygon.Translate(c.X, c.Y),
                                OverlapTolerance))
                        {
                            trueOverlap = true;
                            break;
                        }
                    }
                    if (trueOverlap)
                    {
                        trueGateRejects++;
                        continue;
                    }

                    bestFrame = candEntry.Frame;
                    bestScore = candEntry.Score;
                    bestY = candEntry.Y;
                    bestX = candEntry.X;
                    bestOrient = candEntry.FullOrient;
                    bestContact = candEntry.Contact;
                    bestBboxGrow = candEntry.BboxGrow;
                    break;
                }

                if (bestFrame < 0)
                {
                    // No pending frame has any candidate that passes the true-
                    // polygon gate against the stack; the rest spill into the
                    // fill order.
                    break;
                }

                // Commit. The TRUE-polygon overlap gate already ran in the
                // sorted commit walk above — every committed position is
                // verified on FULL-RESOLUTION geometry, so Frame-pair overlaps
                // stays 0 regardless of proxy drift.
                //
                // `committed` carries the FULL-RES orientation: the
                // PreplacedPart emit below, the fill's seeded sheet state, and
                // the drawn output (holes included, per dafe397) all read from
                // it. Only the scratch scoring state (sheet.Placed forbidden
                // source + placedFramesUnion contact source) uses the proxy.
                committed.Add((bestFrame, bestOrient, bestX, bestY));
                var bestProxyOrient = proxyOrientByIndex[bestOrient.OrientationIndex];
                sheet.Placed.Add(new PlacedItem(bestProxyOrient, bestX, bestY));
                sheet.ForbiddenByOrientation.Clear();
                pending.Remove(bestFrame);

                // Update incremental state: placed-frames PROXY union (contact
                // scoring) + combined FULL-RES bbox.
                placedFramesUnion.Add(ClipperConvert.ToPath64(
                    bestProxyOrient.CanonicalPolygon.Translate(bestX, bestY)));
                placedFramesUnion = Clipper.Union(placedFramesUnion, FillRule.NonZero);

                double bMaxX = bestX + bestOrient.BBox.MaxX;
                double bMaxY = bestY + bestOrient.BBox.MaxY;
                if (!hasCommitted)
                {
                    curMinX = bestX; curMinY = bestY;
                    curMaxX = bMaxX; curMaxY = bMaxY;
                    hasCommitted = true;
                }
                else
                {
                    curMinX = Math.Min(curMinX, bestX);
                    curMinY = Math.Min(curMinY, bestY);
                    curMaxX = Math.Max(curMaxX, bMaxX);
                    curMaxY = Math.Max(curMaxY, bMaxY);
                }
                curHalfPerim = (curMaxX - curMinX) + (curMaxY - curMinY);

                double availForBest = bestAvailContact.TryGetValue(bestFrame, out double av) ? av : 0.0;
                diag.Add($"  Placed frame {bestFrame} at ({bestX:F3},{bestY:F3}) " +
                         $"orient={bestOrient.OrientationIndex} rot={bestOrient.RotationDeg:F0} " +
                         $"mirror={bestOrient.IsMirrored} [committed contact={bestContact:F2} " +
                         $"bboxGrow={bestBboxGrow:F2} score={bestScore:F2}; " +
                         $"best available contact this frame={availForBest:F2}]");
                DiagnosticLog?.Invoke(
                    $"Coordinated frames: placed frame {bestFrame} " +
                    $"({committed.Count} of {totalFrames}).");
            }

            if (pending.Count > 0)
                diag.Add($"  Frames not placed (spilled to fill): " +
                         string.Join(",", pending));

            // Emit committed frames as PreplacedPart using the canonical
            // transform chain (matches NestingEngine's Phase 24b emit).
            var preplaced = new List<PreplacedPart>();
            foreach (var c in committed)
            {
                var src = _request.Polygons[c.Part];
                var srcBBox = src.BoundingBox;
                double rotDeg = c.Orient.RotationDeg;

                var rotatedNormalized = src.MoveToOrigin();
                if (Math.Abs(rotDeg) > 0.01)
                    rotatedNormalized = rotatedNormalized.RotateAround(
                        Point2D.Origin, rotDeg * Math.PI / 180.0);
                var rotBBox = rotatedNormalized.BoundingBox;

                var step1 = Transform2D.Translation(-srcBBox.MinX, -srcBBox.MinY);
                var step2 = Transform2D.RotationDegrees(rotDeg);
                var step3 = Transform2D.Translation(-rotBBox.MinX, -rotBBox.MinY);
                var step4 = Transform2D.Translation(c.X, c.Y);
                var combined = step1.Then(step2).Then(step3).Then(step4);

                var placedPoly = c.Orient.CanonicalPolygon.Translate(c.X, c.Y);

                var placement = new PlacementResult(
                    originalIndex: c.Part,
                    sheet: 0,
                    transform: combined,
                    rotationDeg: rotDeg,
                    isMirrored: c.Orient.IsMirrored,
                    sourceBBoxMinX: srcBBox.MinX,
                    sourceBBoxMaxX: srcBBox.MaxX,
                    placedPolygon: placedPoly);

                preplaced.Add(new PreplacedPart(c.Part, 0, c.Orient, c.X, c.Y, placement));
            }

            framePhaseSw.Stop();
            diag.Add($"Frame-phase time: {framePhaseSw.Elapsed.TotalSeconds:F2}s " +
                     $"({committed.Count}/{totalFrames} frames placed; " +
                     $"proxy scoring, {trueGateRejects} candidate(s) rejected by true-polygon commit gate).");

            // Fill order: caller's remaining parts, then any unplaced frames so
            // they get a normal (possibly spilling) placement attempt.
            var fillOrder = new List<int>(remainingOrder);
            fillOrder.AddRange(pending);

            // Fill runs EXACTLY ONCE — the single PlaceAllWithPreplaced call
            // site in this method. The stopwatch below proves it: if fill were
            // being invoked per-frame somewhere upstream, the reported time
            // would be per-single-invocation (much lower than the caller sees).
            DiagnosticLog?.Invoke(
                $"Coordinated frames: starting single fill phase over {fillOrder.Count} parts...");
            var fillPhaseSw = System.Diagnostics.Stopwatch.StartNew();
            var fillResult = PlaceAllWithPreplaced(fillOrder, preplaced);
            fillPhaseSw.Stop();

            int fillPlaced = fillResult.Placements.Count - preplaced.Count;
            diag.Add($"Fill-phase time: {fillPhaseSw.Elapsed.TotalSeconds:F2}s " +
                     $"(single PlaceAllWithPreplaced call; {fillPlaced}/{fillOrder.Count} fill parts placed).");
            diag.Add($"Total frame+fill time: " +
                     $"{(framePhaseSw.Elapsed + fillPhaseSw.Elapsed).TotalSeconds:F2}s.");

            return fillResult;
        }

        public sealed class NestResult
        {
            public int SheetCount { get; }
            public IReadOnlyList<PlacementResult> Placements { get; }
            public IReadOnlyList<int> Unplaced { get; }

            public NestResult(
                int sheetCount,
                IReadOnlyList<PlacementResult> placements,
                IReadOnlyList<int> unplaced)
            {
                SheetCount = sheetCount;
                Placements = placements;
                Unplaced = unplaced;
            }
        }

        // ------------------------------------------------------------------
        // Placement constants
        // ------------------------------------------------------------------

        private const int TryPlaceOnSheetTimeBudgetSeconds = 30;

        private const double BLVertexSanityFactor = 10.0;

        private const double OverlapTolerance = 1e-4;

        private const int DefaultMaxCandidateVertices = 96;

        public int MaxCandidateVertices { get; set; } = DefaultMaxCandidateVertices;

        private const int MaxInteriorCandidatesPerOrientation = 512;

        // Phase 22b — exact-demand parallel NFP gathering.
        // Below this count, Parallel.For overhead can cost more than it saves.
        private const int ParallelNfpPlacedThreshold = 3;

        // Cap worker count so Rhino remains responsive and Clipper/GC pressure stays sane.
        private const int MaxParallelNfpWorkers = 8;

        // Phase 28.1.5 (TEMPORARY DIAGNOSTIC): when true, the forbidden-region
        // build always runs serially, bypassing BuildForbiddenRegionParallel.
        // Used to localize beam-search nondeterminism: run the benchmark twice
        // with this true (serial) and twice with it false (parallel), diff the
        // logs. If serial is deterministic and parallel isn't, the parallel
        // path is the source despite the clean Lazy/Clipper code review.
        //
        // Default is TRUE for the current diagnostic run. To test the parallel
        // path, set the SEANEST_PARALLEL_FORBIDDEN environment variable to "1"
        // (any non-null/non-empty value flips this to false → parallel).
        // Remove this toggle entirely once the source is localized.
        private static readonly bool ForceSerialForbiddenRegion =
            string.IsNullOrEmpty(
                Environment.GetEnvironmentVariable("SEANEST_PARALLEL_FORBIDDEN"));

        private const double CandidateDuplicateTolerance = 0.001;

        private const double ContactTolerance = 0.02;

        // Score weights. Lower total score is better.
        private const double UsedAreaWeight = 100.0;
        private const double UsedTopWeight = 25.0;
        private const double UsedRightWeight = 5.0;
        private const double BottomBiasWeight = 0.50;
        private const double LeftBiasWeight = 0.10;
        private const double ContactRewardWeight = 30.0;
        private const double LooseIslandPenaltyWeight = 4.0;

        // Phase 28.3 / 28.3.1: weight on regret (feasible-area lost by unplaced
        // critical parts when the current part's candidate is placed). The
        // effective value (_regretWeight) is read once at construction from the
        // SEANEST_REGRET_WEIGHT environment variable, falling back to this
        // default, so the weight can be swept (0, 1, 10, 100, ...) without
        // recompiling.
        //
        // Phase 28.3.2: default is 0.0 (regret OFF). A weight sweep on the
        // 31-part benchmark showed regret a net negative — weight 0 and 1 both
        // reached the best result (8 parts), weight 100 regressed to 7, and
        // weight 10 blew the time budget on per-candidate regret cost. The
        // mechanism stays intact; set SEANEST_REGRET_WEIGHT to re-enable it.
        private const double DefaultRegretWeight = 0.0;

        // Phase 30 increment 3: frame-nester contact metric weights and inflate
        // delta multiplier. Score (lower=better) = w_bbox*bboxGrow - w_contact*contactArea
        // so contact dominates bbox-growth — the cradling reward. Inflate delta
        // = multiplier * spacing so a candidate sitting at the NFP boundary
        // (spacing-distance from a placed frame) overlaps the placed union by
        // ~length × spacing of profile contact. These defaults are overridden at
        // construction by SEANEST_FRAME_CONTACT_W / SEANEST_FRAME_BBOX_W /
        // SEANEST_FRAME_INFLATE_MULT so David can sweep without recompiling
        // (same pattern as SEANEST_REGRET_WEIGHT).
        private const double DefaultFrameContactWeight = 10.0;
        private const double DefaultFrameBboxWeight = 1.0;
        private const double DefaultContactInflateMultiplier = 2.0;

        // ------------------------------------------------------------------
        // Per-sheet placement
        // ------------------------------------------------------------------

        private bool TryPlaceOnSheet(
            int partIndex,
            List<OrientedPart> orientations,
            SheetState sheet,
            int sheetIdx,
            List<PlacementResult> placements)
        {
            BestPlacement best = null;
            int overlapRejections = 0;

            long tForbidden = 0;
            long tFeasible = 0;
            long tCandidateSweep = 0;

            int orientationsTried = 0;
            int orientationsFit = 0;
            int candidateCount = 0;
            int validCandidateCount = 0;

            var sw = new System.Diagnostics.Stopwatch();

            var attemptStopwatch = System.Diagnostics.Stopwatch.StartNew();
            var timeBudget = TimeSpan.FromSeconds(TryPlaceOnSheetTimeBudgetSeconds);

            foreach (var orientation in orientations)
            {
                if (attemptStopwatch.Elapsed > timeBudget)
                {
                    if (best != null)
                    {
                        DiagnosticLog?.Invoke(
                            $"  Part {partIndex} budget exceeded after " +
                            $"{attemptStopwatch.Elapsed.TotalSeconds:F1}s on sheet {sheetIdx} " +
                            $"— using best-so-far " +
                            $"({orientationsTried}/{orientations.Count} orientations checked, " +
                            $"{orientationsFit} fit, {validCandidateCount}/{candidateCount} candidates valid).");
                    }
                    else
                    {
                        DiagnosticLog?.Invoke(
                            $"  Part {partIndex} budget exceeded after " +
                            $"{attemptStopwatch.Elapsed.TotalSeconds:F1}s on sheet {sheetIdx} " +
                            $"— no placement found in time, falling through to next sheet " +
                            $"({orientationsTried}/{orientations.Count} orientations checked, " +
                            $"{orientationsFit} fit, {validCandidateCount}/{candidateCount} candidates valid).");
                    }

                    break;
                }

                orientationsTried++;

                var ifp = InnerFitPolygon.Compute(
                    orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                if (!ifp.HasValue)
                    continue;

                orientationsFit++;

                var ifpBox = ifp.Value;

                sw.Restart();
                Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                sw.Stop();
                tForbidden += sw.ElapsedMilliseconds;

                sw.Restart();
                Paths64 feasible = ComputeFeasibleRegion(ifpBox, forbidden);
                sw.Stop();
                tFeasible += sw.ElapsedMilliseconds;

                if (feasible.Count == 0)
                    continue;

                sw.Restart();
                var candidates = FindCandidateVertices(
                    feasible,
                    MaxCandidateVertices);
                sw.Stop();
                tCandidateSweep += sw.ElapsedMilliseconds;

                if (candidates.Count == 0)
                    continue;

                candidateCount += candidates.Count;

                foreach (var candidate in candidates)
                {
                    double tx = candidate.X;
                    double ty = candidate.Y;

                    if (!IsSaneCandidate(tx, ty))
                        continue;

                    var candidatePoly = orientation.CanonicalPolygon.Translate(tx, ty);

                    bool rejected = false;

                    foreach (var placed in sheet.Placed)
                    {
                        var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                            placed.X,
                            placed.Y);

                        if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                        {
                            overlapRejections++;
                            rejected = true;
                            break;
                        }
                    }

                    if (rejected)
                        continue;

                    validCandidateCount++;

                    var score = ScorePlacementCandidate(candidatePoly, sheet);

                    if (best == null || IsBetterPlacement(score, tx, ty, best))
                    {
                        best = new BestPlacement
                        {
                            X = tx,
                            Y = ty,
                            Orientation = orientation,
                            Score = score
                        };
                    }
                }
            }

            // Phase 26: interior sampling fallback. When boundary vertices all
            // fail overlap check (best == null, overlapRejections > 0), generate
            // grid points INSIDE the feasible region where there's actual clearance.
            int interiorCandidateCount = 0;
            int interiorValidCount = 0;
            bool interiorFallbackFired = false;

            if (best == null && EnableInteriorSampling && overlapRejections > 0)
            {
                interiorFallbackFired = true;
                double step = InteriorSamplingStep ??
                    Math.Max(1.0, _request.Spacing * 3.0);

                foreach (var orientation in orientations)
                {
                    if (attemptStopwatch.Elapsed > timeBudget)
                        break;

                    var ifp = InnerFitPolygon.Compute(
                        orientation,
                        _request.SheetWidth,
                        _request.SheetHeight,
                        _request.Margin);

                    if (!ifp.HasValue)
                        continue;

                    Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                    Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);

                    if (feasible.Count == 0)
                        continue;

                    var interiorPoints = SampleInteriorPoints(feasible, step);
                    if (interiorPoints.Count == 0)
                        continue;

                    interiorCandidateCount += interiorPoints.Count;

                    foreach (var pt in interiorPoints)
                    {
                        if (!IsSaneCandidate(pt.X, pt.Y))
                            continue;

                        var candidatePoly = orientation.CanonicalPolygon.Translate(pt.X, pt.Y);

                        bool rejected = false;
                        foreach (var placed in sheet.Placed)
                        {
                            var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                                placed.X, placed.Y);
                            if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                            {
                                rejected = true;
                                break;
                            }
                        }

                        if (rejected)
                            continue;

                        interiorValidCount++;

                        var score = ScorePlacementCandidate(candidatePoly, sheet);

                        if (best == null || IsBetterPlacement(score, pt.X, pt.Y, best))
                        {
                            best = new BestPlacement
                            {
                                X = pt.X,
                                Y = pt.Y,
                                Orientation = orientation,
                                Score = score
                            };
                        }
                    }
                }
            }

            if (interiorFallbackFired)
            {
                InteriorFallbackLog.Add(
                    $"Part {partIndex} sheet {sheetIdx}: boundary failed (overlapRejects={overlapRejections}), " +
                    $"interior fallback: {interiorValidCount}/{interiorCandidateCount} valid, " +
                    $"best={(best != null ? $"yes pos=({best.X:F3},{best.Y:F3})" : "no")}");
            }

            string interiorInfo = interiorFallbackFired
                ? $", interior fallback: {interiorValidCount}/{interiorCandidateCount} valid"
                : "";

            DiagnosticLog?.Invoke(
                $"  Part {partIndex} sheet {sheetIdx}: " +
                $"orient {orientationsFit}/{orientationsTried}, " +
                $"placed {sheet.Placed.Count}, " +
                $"cache {_nfpCache.Count}, " +
                $"forbidden {tForbidden}ms, " +
                $"feasible {tFeasible}ms, " +
                $"candidates {validCandidateCount}/{candidateCount}, " +
                $"sweep {tCandidateSweep}ms, " +
                $"overlapRejects {overlapRejections}{interiorInfo}, " +
                $"best={(best != null ? $"yes orient={best.Orientation.OrientationIndex} pos=({best.X:F3},{best.Y:F3}) score={best.Score.Total:F3}" : "no")}");

            if (best == null)
                return false;

            double bxLimit = BLVertexSanityFactor * _request.SheetWidth;
            double byLimit = BLVertexSanityFactor * _request.SheetHeight;

            if (Math.Abs(best.X) > bxLimit || Math.Abs(best.Y) > byLimit)
            {
                var candBBox = best.Orientation.CanonicalPolygon.BoundingBox;

                var sanityIfp = InnerFitPolygon.Compute(
                    best.Orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                string ifpStr = sanityIfp.HasValue
                    ? $"[{sanityIfp.Value.MinX:F3},{sanityIfp.Value.MinY:F3}]-[{sanityIfp.Value.MaxX:F3},{sanityIfp.Value.MaxY:F3}]"
                    : "(no-fit)";

                throw new InvalidOperationException(
                    $"NFP placement produced out-of-bounds candidate ({best.X:G6}, {best.Y:G6}) " +
                    $"for part {partIndex}, orientation {best.Orientation.OrientationIndex} " +
                    $"(rot={best.Orientation.RotationDeg:F0}{(best.Orientation.IsMirrored ? "m" : "")}, " +
                    $"src-part={best.Orientation.SourcePartIndex}), sheet {sheetIdx}. " +
                    $"Sheet is {_request.SheetWidth}x{_request.SheetHeight}; " +
                    $"limit is {BLVertexSanityFactor}x sheet dimensions. " +
                    $"Candidate canonical bbox=[{candBBox.MinX:F3},{candBBox.MinY:F3}]-[{candBBox.MaxX:F3},{candBBox.MaxY:F3}]; " +
                    $"IFP={ifpStr}. Aborting nest.");
            }

            var placedPolygon = best.Orientation.CanonicalPolygon.Translate(best.X, best.Y);

            double rotRad = best.Orientation.RotationDeg * Math.PI / 180.0;

            var sourcePoly = _request.Polygons[partIndex];
            var srcBBox = sourcePoly.BoundingBox;

            var rotatedNormalized = sourcePoly
                .MoveToOrigin()
                .RotateAround(Point2D.Origin, rotRad);

            var rotBBox = rotatedNormalized.BoundingBox;

            var step1 = Transform2D.Translation(-srcBBox.MinX, -srcBBox.MinY);
            var step2 = Transform2D.RotationDegrees(best.Orientation.RotationDeg);
            var step3 = Transform2D.Translation(-rotBBox.MinX, -rotBBox.MinY);
            var step4 = Transform2D.Translation(best.X, best.Y);

            var combined = step1.Then(step2).Then(step3).Then(step4);

            placements.Add(new PlacementResult(
                originalIndex: partIndex,
                sheet: sheetIdx,
                transform: combined,
                rotationDeg: best.Orientation.RotationDeg,
                isMirrored: best.Orientation.IsMirrored,
                sourceBBoxMinX: srcBBox.MinX,
                sourceBBoxMaxX: srcBBox.MaxX,
                placedPolygon: placedPolygon));

            sheet.Placed.Add(new PlacedItem(best.Orientation, best.X, best.Y));

            // The forbidden region depends on every placed part. Clear it after
            // each placement so the next part rebuilds against the updated sheet.
            sheet.ForbiddenByOrientation.Clear();

            return true;
        }

        // ------------------------------------------------------------------
        // Geometry steps
        // ------------------------------------------------------------------

        private Paths64 GetOrBuildForbiddenRegion(
            OrientedPart candidate,
            SheetState sheet)
        {
            if (sheet.Placed.Count == 0)
                return new Paths64();

            if (sheet.ForbiddenByOrientation.TryGetValue(
                    candidate.OrientationIndex,
                    out var cached))
            {
                return cached;
            }

            Paths64 allNfps;

            if (!ForceSerialForbiddenRegion && sheet.Placed.Count >= ParallelNfpPlacedThreshold)
            {
                allNfps = BuildForbiddenRegionParallel(candidate, sheet);
            }
            else
            {
                allNfps = BuildForbiddenRegionSerial(candidate, sheet);
            }

            // FillRule.NonZero per codebase precedent (matches Phase 21b shipped behavior).
            // This phase is a speed patch only; do not change the fill rule.
            Paths64 union = allNfps.Count == 0
                ? new Paths64()
                : Clipper.Union(allNfps, FillRule.NonZero);

            sheet.ForbiddenByOrientation[candidate.OrientationIndex] = union;
            return union;
        }

        private Paths64 BuildForbiddenRegionSerial(
            OrientedPart candidate,
            SheetState sheet)
        {
            var allNfps = new Paths64();

            foreach (var placed in sheet.Placed)
            {
                var nfp = _nfpCache.Get(placed.Orientation, candidate);

                foreach (var nfpPoly in nfp)
                {
                    var translated = nfpPoly.Translate(placed.X, placed.Y);
                    allNfps.Add(ClipperConvert.ToPath64(translated));
                }
            }

            return allNfps;
        }

        private Paths64 BuildForbiddenRegionParallel(
            OrientedPart candidate,
            SheetState sheet)
        {
            int placedCount = sheet.Placed.Count;
            var perPlacedPaths = new Paths64[placedCount];

            int maxWorkers = GetMaxNfpParallelism();

            var options = new ParallelOptions
            {
                MaxDegreeOfParallelism = maxWorkers
            };

            Parallel.For(
                0,
                placedCount,
                options,
                i =>
                {
                    var placed = sheet.Placed[i];
                    var localPaths = new Paths64();

                    var nfp = _nfpCache.Get(placed.Orientation, candidate);

                    foreach (var nfpPoly in nfp)
                    {
                        var translated = nfpPoly.Translate(placed.X, placed.Y);
                        localPaths.Add(ClipperConvert.ToPath64(translated));
                    }

                    perPlacedPaths[i] = localPaths;
                });

            // Merge in deterministic placed-order after the parallel work completes.
            var allNfps = new Paths64();

            for (int i = 0; i < perPlacedPaths.Length; i++)
            {
                var local = perPlacedPaths[i];
                if (local == null || local.Count == 0)
                    continue;

                for (int p = 0; p < local.Count; p++)
                    allNfps.Add(local[p]);
            }

            return allNfps;
        }

        private static int GetMaxNfpParallelism()
        {
            int cpu = Environment.ProcessorCount;
            int workers = Math.Max(1, cpu - 1);
            return Math.Min(MaxParallelNfpWorkers, workers);
        }

        private Paths64 ComputeFeasibleRegion(BoundingBox2D ifp, Paths64 forbidden)
        {
            var ifpPath = new Path64(4);

            long minX = (long)(ifp.MinX * ClipperConvert.Scale);
            long minY = (long)(ifp.MinY * ClipperConvert.Scale);
            long maxX = (long)(ifp.MaxX * ClipperConvert.Scale);
            long maxY = (long)(ifp.MaxY * ClipperConvert.Scale);

            // CCW rectangle: BL, BR, TR, TL.
            ifpPath.Add(new Point64(minX, minY));
            ifpPath.Add(new Point64(maxX, minY));
            ifpPath.Add(new Point64(maxX, maxY));
            ifpPath.Add(new Point64(minX, maxY));

            var subject = new Paths64 { ifpPath };

            if (forbidden.Count == 0)
                return subject;

            // FillRule.NonZero matches the codebase pattern from OverlapChecker.
            return Clipper.Difference(subject, forbidden, FillRule.NonZero);
        }

        private List<(double X, double Y)> FindCandidateVertices(
            Paths64 feasible,
            int maxCount)
        {
            var result = new List<(double X, double Y)>();

            foreach (var path in feasible)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    double x = path[i].X / ClipperConvert.Scale;
                    double y = path[i].Y / ClipperConvert.Scale;

                    bool duplicate = false;

                    for (int j = 0; j < result.Count; j++)
                    {
                        if (Math.Abs(result[j].X - x) <= CandidateDuplicateTolerance &&
                            Math.Abs(result[j].Y - y) <= CandidateDuplicateTolerance)
                        {
                            duplicate = true;
                            break;
                        }
                    }

                    if (!duplicate)
                        result.Add((x, y));
                }
            }

            // Keep the lower-left-biased candidates first so we do not spend
            // time scoring thousands of far-away vertices on complicated sheets.
            result.Sort((a, b) =>
            {
                int yCmp = a.Y.CompareTo(b.Y);
                if (yCmp != 0) return yCmp;
                return a.X.CompareTo(b.X);
            });

            if (maxCount > 0 && result.Count > maxCount)
                result.RemoveRange(maxCount, result.Count - maxCount);

            return result;
        }

        private bool IsSaneCandidate(double x, double y)
        {
            double bxLimit = BLVertexSanityFactor * _request.SheetWidth;
            double byLimit = BLVertexSanityFactor * _request.SheetHeight;

            if (double.IsNaN(x) || double.IsNaN(y)) return false;
            if (double.IsInfinity(x) || double.IsInfinity(y)) return false;

            return Math.Abs(x) <= bxLimit && Math.Abs(y) <= byLimit;
        }

        private static void FlushInteriorFallbackLog()
        {
            if (InteriorFallbackLog.Count == 0) return;

            try
            {
                var lines = new List<string>
                {
                    $"Phase 26 interior fallback log written at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                    ""
                };
                lines.AddRange(InteriorFallbackLog);
                lines.Add("");
                lines.Add($"Total interior fallback events: {InteriorFallbackLog.Count}");
                File.WriteAllLines(InteriorFallbackLogPath, lines);
            }
            catch { /* best effort */ }
        }

        private static List<(double X, double Y)> SampleInteriorPoints(
            Paths64 feasible,
            double step)
        {
            if (feasible.Count == 0 || step <= 0)
                return new List<(double, double)>();

            long scaledStep = (long)(step * ClipperConvert.Scale);
            if (scaledStep < 1) scaledStep = 1;

            long minX = long.MaxValue, minY = long.MaxValue;
            long maxX = long.MinValue, maxY = long.MinValue;

            foreach (var path in feasible)
            {
                for (int i = 0; i < path.Count; i++)
                {
                    if (path[i].X < minX) minX = path[i].X;
                    if (path[i].Y < minY) minY = path[i].Y;
                    if (path[i].X > maxX) maxX = path[i].X;
                    if (path[i].Y > maxY) maxY = path[i].Y;
                }
            }

            if (minX >= maxX || minY >= maxY)
                return new List<(double, double)>();

            long halfStep = scaledStep / 2;
            var result = new List<(double X, double Y)>();

            for (long gy = minY + halfStep; gy <= maxY; gy += scaledStep)
            {
                for (long gx = minX + halfStep; gx <= maxX; gx += scaledStep)
                {
                    var pt = new Point64(gx, gy);

                    if (IsInsideFeasibleRegion(pt, feasible))
                    {
                        result.Add((gx / ClipperConvert.Scale, gy / ClipperConvert.Scale));
                    }

                    if (result.Count >= MaxInteriorCandidatesPerOrientation)
                        break;
                }

                if (result.Count >= MaxInteriorCandidatesPerOrientation)
                    break;
            }

            return result;
        }

        private static bool IsInsideFeasibleRegion(Point64 pt, Paths64 feasible)
        {
            int windingSum = 0;

            foreach (var path in feasible)
            {
                var pip = Clipper.PointInPolygon(pt, path);
                if (pip == PointInPolygonResult.IsOn)
                    return false;
                if (pip == PointInPolygonResult.IsInside)
                {
                    double area = Clipper.Area(path);
                    if (area > 0) windingSum++;
                    else windingSum--;
                }
            }

            return windingSum > 0;
        }


        // ------------------------------------------------------------------
        // Phase 27: single-sheet constrained beam search
        // ------------------------------------------------------------------

        private static readonly string BeamLogPath =
            Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.DesktopDirectory),
                "phase27_beam.txt");

        private struct ScoredCandidate
        {
            public OrientedPart Orientation;
            public double X;
            public double Y;
            public PlacementScore Score;
        }

        // Phase 28.2.3: shared geometry pipeline (IFP -> forbidden region ->
        // feasible region), computed in one place so CountValidPlacements and
        // MeasureFeasibleArea do not duplicate it. Yields per orientation in
        // orientation order; orientations with no IFP or an empty feasible
        // region are skipped.
        private IEnumerable<(OrientedPart Orientation, Paths64 Feasible)>
            EnumerateFeasibleRegions(int partIndex, SheetState sheet)
        {
            var orientations = _orientationsByPart[partIndex];

            foreach (var orientation in orientations)
            {
                var ifp = InnerFitPolygon.Compute(
                    orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                if (!ifp.HasValue)
                    continue;

                Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);

                if (feasible.Count == 0)
                    continue;

                yield return (orientation, feasible);
            }
        }

        private int CountValidPlacements(
            int partIndex,
            SheetState sheet,
            int cap)
        {
            int count = 0;

            foreach (var (orientation, feasible) in
                EnumerateFeasibleRegions(partIndex, sheet))
            {
                var candidates = FindCandidateVertices(feasible, BeamMaxCandidates);

                foreach (var cand in candidates)
                {
                    if (!IsSaneCandidate(cand.X, cand.Y))
                        continue;

                    var candidatePoly = orientation.CanonicalPolygon.Translate(cand.X, cand.Y);

                    bool rejected = false;
                    foreach (var placed in sheet.Placed)
                    {
                        var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                            placed.X, placed.Y);
                        if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                        {
                            rejected = true;
                            break;
                        }
                    }

                    if (!rejected)
                    {
                        count++;
                        if (count >= cap) return count;
                    }
                }

                if (EnableInteriorSampling)
                {
                    double step = InteriorSamplingStep ??
                        Math.Max(1.0, _request.Spacing * 3.0);

                    var interiorPoints = SampleInteriorPoints(feasible, step);

                    foreach (var pt in interiorPoints)
                    {
                        if (!IsSaneCandidate(pt.X, pt.Y))
                            continue;

                        var candidatePoly = orientation.CanonicalPolygon.Translate(pt.X, pt.Y);

                        bool rejected = false;
                        foreach (var placed in sheet.Placed)
                        {
                            var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                                placed.X, placed.Y);
                            if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                            {
                                rejected = true;
                                break;
                            }
                        }

                        if (!rejected)
                        {
                            count++;
                            if (count >= cap) return count;
                        }
                    }
                }
            }

            return count;
        }

        private bool HasAnyValidPlacement(
            int partIndex,
            SheetState sheet)
        {
            return CountValidPlacements(partIndex, sheet, 1) > 0;
        }

        // Phase 28.2.3: feasible-region area metric for MRV decisions. Sums the
        // signed Clipper area of every feasible-region polygon across all
        // orientations and returns the total in model square units. Unlike
        // CountValidPlacements this neither enumerates candidate vertices nor
        // tests overlaps, so it always discriminates (continuous measure) and
        // is cheaper to compute.
        private double MeasureFeasibleArea(
            int partIndex,
            SheetState sheet)
        {
            double scaledArea = 0;

            foreach (var (_, feasible) in
                EnumerateFeasibleRegions(partIndex, sheet))
            {
                foreach (var path in feasible)
                    scaledArea += Clipper.Area(path);
            }

            return scaledArea / (ClipperConvert.Scale * ClipperConvert.Scale);
        }

        private List<ScoredCandidate> GenerateTopKCandidates(
            int partIndex,
            List<OrientedPart> orientations,
            SheetState sheet,
            int maxCandidates,
            int topK,
            out int boundaryValidCount,
            out int interiorValidCount)
        {
            var all = new List<ScoredCandidate>();
            boundaryValidCount = 0;
            interiorValidCount = 0;

            foreach (var orientation in orientations)
            {
                var ifp = InnerFitPolygon.Compute(
                    orientation,
                    _request.SheetWidth,
                    _request.SheetHeight,
                    _request.Margin);

                if (!ifp.HasValue)
                    continue;

                Paths64 forbidden = GetOrBuildForbiddenRegion(orientation, sheet);
                Paths64 feasible = ComputeFeasibleRegion(ifp.Value, forbidden);

                if (feasible.Count == 0)
                    continue;

                // Boundary vertex candidates.
                var candidates = FindCandidateVertices(feasible, maxCandidates);

                foreach (var cand in candidates)
                {
                    if (!IsSaneCandidate(cand.X, cand.Y))
                        continue;

                    var candidatePoly = orientation.CanonicalPolygon.Translate(cand.X, cand.Y);

                    bool rejected = false;
                    foreach (var placed in sheet.Placed)
                    {
                        var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                            placed.X, placed.Y);
                        if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                        {
                            rejected = true;
                            break;
                        }
                    }

                    if (rejected)
                        continue;

                    boundaryValidCount++;
                    var score = ScorePlacementCandidate(candidatePoly, sheet);
                    all.Add(new ScoredCandidate
                    {
                        Orientation = orientation,
                        X = cand.X,
                        Y = cand.Y,
                        Score = score
                    });
                }

                // Phase 28.0: interior sampling for beam candidate generation.
                // Same logic as Phase 26's fallback in TryPlaceOnSheet, but
                // runs unconditionally (not as a fallback) to maximize the
                // candidate pool before K-truncation.
                if (EnableInteriorSampling)
                {
                    double step = InteriorSamplingStep ??
                        Math.Max(1.0, _request.Spacing * 3.0);

                    var interiorPoints = SampleInteriorPoints(feasible, step);

                    foreach (var pt in interiorPoints)
                    {
                        if (!IsSaneCandidate(pt.X, pt.Y))
                            continue;

                        var candidatePoly = orientation.CanonicalPolygon.Translate(pt.X, pt.Y);

                        bool rejected = false;
                        foreach (var placed in sheet.Placed)
                        {
                            var priorPoly = placed.Orientation.CanonicalPolygon.Translate(
                                placed.X, placed.Y);
                            if (OverlapChecker.Overlaps(candidatePoly, priorPoly, OverlapTolerance))
                            {
                                rejected = true;
                                break;
                            }
                        }

                        if (rejected)
                            continue;

                        interiorValidCount++;
                        var score = ScorePlacementCandidate(candidatePoly, sheet);
                        all.Add(new ScoredCandidate
                        {
                            Orientation = orientation,
                            X = pt.X,
                            Y = pt.Y,
                            Score = score
                        });
                    }
                }
            }

            // Phase 28.1.5: total-order sort. List.Sort (introsort) is
            // unstable, so any ties at Score.Total would leave the order of
            // distinct candidates undefined and break byte-identical output.
            // The ladder below is provably total for distinct candidates:
            // two candidates with identical orientation index AND identical
            // X AND identical Y (full double precision) are the same physical
            // placement, so comparing equal is correct.
            all.Sort((a, b) =>
            {
                int cmp = a.Score.Total.CompareTo(b.Score.Total);
                if (cmp != 0) return cmp;
                cmp = a.Score.UsedArea.CompareTo(b.Score.UsedArea);
                if (cmp != 0) return cmp;
                cmp = a.Score.UsedTop.CompareTo(b.Score.UsedTop);
                if (cmp != 0) return cmp;
                cmp = a.Score.UsedRight.CompareTo(b.Score.UsedRight);
                if (cmp != 0) return cmp;
                cmp = a.Orientation.OrientationIndex.CompareTo(b.Orientation.OrientationIndex);
                if (cmp != 0) return cmp;
                cmp = a.X.CompareTo(b.X);
                if (cmp != 0) return cmp;
                return a.Y.CompareTo(b.Y);
            });
            if (all.Count > topK)
                all.RemoveRange(topK, all.Count - topK);

            return all;
        }

        private sealed class BeamState
        {
            public SheetState Sheet;
            public List<PlacementResult> Placements;
            public double TotalScore;
            public double MaxUsedTop;
            public double MaxUsedRight;

            // Phase 28.3: weighted regret term applied to the candidate that
            // produced this state. Diagnostic only — not used by the sort.
            public double LastRegret;
        }

        public NestResult TrySingleSheetBeamPack(
            IReadOnlyList<int> partOrder,
            TimeSpan budget,
            int K,
            int B)
        {
            if (partOrder == null || partOrder.Count == 0)
                return null;

            // Phase 28.2: mutable copy of order for MRV reordering.
            var mutableOrder = new List<int>(partOrder);

            var sw = Stopwatch.StartNew();
            var log = new List<string>
            {
                $"Phase 27 beam search run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}",
                $"Parts: {mutableOrder.Count}, B={B}, K={K}, maxCandidates={BeamMaxCandidates}, budget={budget.TotalSeconds:F0}s",
                $"Order: {string.Join(",", mutableOrder)}",
                ""
            };

            var initialState = new BeamState
            {
                Sheet = new SheetState(),
                Placements = new List<PlacementResult>(),
                TotalScore = 0,
                MaxUsedTop = 0,
                MaxUsedRight = 0
            };

            var beam = new List<BeamState> { initialState };

            for (int step = 0; step < mutableOrder.Count; step++)
            {
                if (sw.Elapsed >= budget)
                {
                    log.Add($"Part {mutableOrder[step]} (step {step}): time budget exceeded at {sw.ElapsedMilliseconds}ms. Aborting.");
                    FlushBeamLog(log);
                    return null;
                }

                // Phase 28.2: MRV reordering within Irregular tier.
                if (IrregularPartIndices != null &&
                    IrregularPartIndices.Contains(mutableOrder[step]))
                {
                    // Count unplaced Irregulars from this step onward.
                    var unplacedIrregulars = new List<int>();
                    for (int j = step; j < mutableOrder.Count; j++)
                    {
                        if (IrregularPartIndices.Contains(mutableOrder[j]))
                            unplacedIrregulars.Add(mutableOrder[j]);
                    }

                    if (unplacedIrregulars.Count >= 2 && beam.Count > 0)
                    {
                        int bestIdx = unplacedIrregulars[0];
                        double bestMinArea = double.MaxValue;
                        double bestBBoxArea = 0;
                        var mrvDetails = new List<string>();

                        foreach (int ui in unplacedIrregulars)
                        {
                            double minArea = double.MaxValue;
                            foreach (var state in beam)
                            {
                                double a = MeasureFeasibleArea(ui, state.Sheet);
                                if (a < minArea) minArea = a;
                            }

                            var bb = _request.Polygons[ui].BoundingBox;
                            double bboxArea = bb.Width * bb.Height;
                            mrvDetails.Add($"{ui}={minArea:F1}");

                            if (minArea < bestMinArea ||
                                (minArea == bestMinArea && bboxArea > bestBBoxArea))
                            {
                                bestIdx = ui;
                                bestMinArea = minArea;
                                bestBBoxArea = bboxArea;
                            }
                        }

                        log.Add($"Part {mutableOrder[step]} (step {step}): MRV evaluated [{string.Join(", ", mrvDetails)}], picked Part {bestIdx}");

                        if (bestIdx != mutableOrder[step])
                        {
                            int swapFrom = mutableOrder.IndexOf(bestIdx);
                            if (swapFrom > step)
                            {
                                mutableOrder.RemoveAt(swapFrom);
                                mutableOrder.Insert(step, bestIdx);
                            }
                        }
                    }
                }

                int partIndex = mutableOrder[step];
                var orientations = _orientationsByPart[partIndex];
                var nextBeam = new List<BeamState>();

                // Build set of already-placed indices for future-domain check.
                var placedIndicesSet = new HashSet<int>();
                if (beam.Count > 0)
                {
                    foreach (var pp in beam[0].Placements)
                        placedIndicesSet.Add(pp.OriginalIndex);
                }
                placedIndicesSet.Add(partIndex);

                // Identify unplaced critical parts for future-domain pruning.
                List<int> unplacedCriticals = null;
                if (CriticalPartIndices != null && CriticalPartIndices.Count > 0)
                {
                    unplacedCriticals = new List<int>();
                    foreach (int ci in CriticalPartIndices)
                    {
                        if (!placedIndicesSet.Contains(ci))
                            unplacedCriticals.Add(ci);
                    }
                    if (unplacedCriticals.Count == 0)
                        unplacedCriticals = null;
                }

                int stateIdx = 0;
                int prunedByFutureDomain = 0;

                foreach (var state in beam)
                {
                    var topK = GenerateTopKCandidates(
                        partIndex, orientations, state.Sheet,
                        BeamMaxCandidates, K,
                        out int bValid, out int iValid);

                    // Phase 28.3: feasible area of each unplaced critical part
                    // BEFORE placing the current part. Independent of which
                    // candidate is chosen, so compute once per beam state.
                    double[] criticalAreaBefore = null;
                    if (unplacedCriticals != null)
                    {
                        criticalAreaBefore = new double[unplacedCriticals.Count];
                        for (int c = 0; c < unplacedCriticals.Count; c++)
                            criticalAreaBefore[c] =
                                MeasureFeasibleArea(unplacedCriticals[c], state.Sheet);
                    }

                    // Phase 28.3: track this state's best (lowest-adjusted)
                    // surviving candidate for the diagnostic log.
                    bool regretLogged = false;
                    double bestRawScore = 0;
                    double bestRegretTerm = 0;
                    double bestAdjusted = double.MaxValue;

                    foreach (var cand in topK)
                    {
                        var newSheet = state.Sheet.Clone();
                        newSheet.Placed.Add(new PlacedItem(cand.Orientation, cand.X, cand.Y));
                        newSheet.ForbiddenByOrientation.Clear();

                        // Phase 28.1: future-domain pruning. Check that each
                        // unplaced critical part still has at least one legal
                        // placement in this candidate state.
                        bool prunedByFD = false;
                        if (unplacedCriticals != null)
                        {
                            foreach (int ci in unplacedCriticals)
                            {
                                if (!HasAnyValidPlacement(ci, newSheet))
                                {
                                    prunedByFutureDomain++;
                                    prunedByFD = true;
                                    log.Add($"  Part {partIndex} state {stateIdx} candidate ({cand.X:F1},{cand.Y:F1}): " +
                                            $"PRUNED — critical Part {ci} has 0 legal placements");
                                    break;
                                }
                            }
                        }

                        if (prunedByFD)
                            continue;

                        // Phase 28.3: regret — total feasible area the unplaced
                        // critical parts lose when this candidate is placed.
                        // Future-domain pruning above already populated newSheet's
                        // forbidden cache for the criticals, so the "after"
                        // measurements mostly reuse it.
                        double regretTerm = 0;
                        if (criticalAreaBefore != null)
                        {
                            double totalRegret = 0;
                            for (int c = 0; c < unplacedCriticals.Count; c++)
                            {
                                double after =
                                    MeasureFeasibleArea(unplacedCriticals[c], newSheet);
                                totalRegret += criticalAreaBefore[c] - after;
                            }
                            regretTerm = _regretWeight * totalRegret;
                        }

                        var placedPoly = cand.Orientation.CanonicalPolygon.Translate(cand.X, cand.Y);

                        double rotRad = cand.Orientation.RotationDeg * Math.PI / 180.0;
                        var sourcePoly = _request.Polygons[partIndex];
                        var srcBBox = sourcePoly.BoundingBox;
                        var rotatedNorm = sourcePoly.MoveToOrigin();
                        if (Math.Abs(cand.Orientation.RotationDeg) > 0.01)
                            rotatedNorm = rotatedNorm.RotateAround(Point2D.Origin, rotRad);
                        var rotBBox = rotatedNorm.BoundingBox;

                        var step1 = Transform2D.Translation(-srcBBox.MinX, -srcBBox.MinY);
                        var step2 = Transform2D.RotationDegrees(cand.Orientation.RotationDeg);
                        var step3 = Transform2D.Translation(-rotBBox.MinX, -rotBBox.MinY);
                        var step4 = Transform2D.Translation(cand.X, cand.Y);
                        var combined = step1.Then(step2).Then(step3).Then(step4);

                        var newPlacements = new List<PlacementResult>(state.Placements);
                        newPlacements.Add(new PlacementResult(
                            originalIndex: partIndex,
                            sheet: 0,
                            transform: combined,
                            rotationDeg: cand.Orientation.RotationDeg,
                            isMirrored: cand.Orientation.IsMirrored,
                            sourceBBoxMinX: srcBBox.MinX,
                            sourceBBoxMaxX: srcBBox.MaxX,
                            placedPolygon: placedPoly));

                        var placedBox = placedPoly.BoundingBox;
                        double newMaxTop = Math.Max(state.MaxUsedTop, placedBox.MaxY);
                        double newMaxRight = Math.Max(state.MaxUsedRight, placedBox.MaxX);

                        nextBeam.Add(new BeamState
                        {
                            Sheet = newSheet,
                            Placements = newPlacements,
                            TotalScore = state.TotalScore + cand.Score.Total + regretTerm,
                            MaxUsedTop = newMaxTop,
                            MaxUsedRight = newMaxRight,
                            LastRegret = regretTerm
                        });

                        // Phase 28.3: remember this state's best surviving
                        // candidate for the per-state diagnostic line.
                        double adjustedIncremental = cand.Score.Total + regretTerm;
                        if (adjustedIncremental < bestAdjusted)
                        {
                            bestAdjusted = adjustedIncremental;
                            bestRawScore = cand.Score.Total;
                            bestRegretTerm = regretTerm;
                            regretLogged = true;
                        }
                    }

                    if (EnableInteriorSampling && (bValid > 0 || iValid > 0))
                    {
                        log.Add($"  Part {partIndex} state {stateIdx}: boundary={bValid} valid, interior={iValid} valid, topK={topK.Count}");
                    }

                    // Phase 28.3: per-state regret diagnostic.
                    if (criticalAreaBefore != null && regretLogged)
                    {
                        log.Add($"  Part {partIndex} state {stateIdx}: best candidate " +
                                $"raw_score={bestRawScore:F1}, regret={bestRegretTerm:F1}, " +
                                $"adjusted={bestAdjusted:F1}, picked from K={topK.Count}");
                    }
                    stateIdx++;
                }

                if (nextBeam.Count == 0)
                {
                    log.Add($"Part {partIndex} (step {step}): beam dropped to zero — no state could place on single sheet. Aborting.");
                    FlushBeamLog(log);
                    return null;
                }

                // Phase 28.1.5: total-order sort. After the three score keys,
                // tiebreak on the placement sequence (OriginalIndex, X, Y of
                // each placement in placement order, compared lexically). Two
                // states with identical placements in identical positions are
                // genuinely the same state and may compare equal — that's
                // correct, not a determinism leak.
                nextBeam.Sort((a, b) =>
                {
                    int cmp = a.TotalScore.CompareTo(b.TotalScore);
                    if (cmp != 0) return cmp;
                    cmp = a.MaxUsedTop.CompareTo(b.MaxUsedTop);
                    if (cmp != 0) return cmp;
                    cmp = a.MaxUsedRight.CompareTo(b.MaxUsedRight);
                    if (cmp != 0) return cmp;
                    return ComparePlacementSequence(a.Placements, b.Placements);
                });

                if (nextBeam.Count > B)
                    nextBeam.RemoveRange(B, nextBeam.Count - B);

                string fdInfo = prunedByFutureDomain > 0 ? $" (futureDomain={prunedByFutureDomain})" : "";

                // Phase 28.3: average weighted regret across surviving states.
                double avgRegret = 0;
                foreach (var s in nextBeam) avgRegret += s.LastRegret;
                avgRegret /= nextBeam.Count;

                log.Add($"Part {partIndex} (step {step}): beam {beam.Count} -> branches {beam.Count * K} -> survived {nextBeam.Count}{fdInfo}, " +
                        $"best score {nextBeam[0].TotalScore:F1}, avgRegret {avgRegret:F1}, elapsed {sw.ElapsedMilliseconds}ms");

                beam = nextBeam;
            }

            var best = beam[0];
            log.Add("");
            log.Add($"Phase 27: beam search succeeded — all {mutableOrder.Count} parts on 1 sheet. " +
                    $"Total score {best.TotalScore:F1}, elapsed {sw.ElapsedMilliseconds}ms.");
            FlushBeamLog(log);

            return new NestResult(1, best.Placements, new List<int>());
        }

        private static void FlushBeamLog(List<string> lines)
        {
            try { File.WriteAllLines(BeamLogPath, lines); }
            catch { /* best effort */ }
        }

        // Phase 28.1.5: deterministic lexical comparison of two placement
        // sequences. Compares element-by-element on (OriginalIndex, placed
        // bbox MinX, placed bbox MinY, RotationDeg). Shorter sequence sorts
        // first if it's a prefix. Returns 0 only when the sequences are
        // genuinely identical (same parts, same positions, same order).
        private static int ComparePlacementSequence(
            List<PlacementResult> a, List<PlacementResult> b)
        {
            int n = Math.Min(a.Count, b.Count);
            for (int i = 0; i < n; i++)
            {
                int cmp = a[i].OriginalIndex.CompareTo(b[i].OriginalIndex);
                if (cmp != 0) return cmp;

                var ba = a[i].PlacedPolygon.BoundingBox;
                var bb = b[i].PlacedPolygon.BoundingBox;
                cmp = ba.MinX.CompareTo(bb.MinX);
                if (cmp != 0) return cmp;
                cmp = ba.MinY.CompareTo(bb.MinY);
                if (cmp != 0) return cmp;
                cmp = a[i].RotationDeg.CompareTo(b[i].RotationDeg);
                if (cmp != 0) return cmp;
            }
            return a.Count.CompareTo(b.Count);
        }
        // ------------------------------------------------------------------
        // Smart candidate scoring
        // ------------------------------------------------------------------

        private PlacementScore ScorePlacementCandidate(
            Polygon candidatePoly,
            SheetState sheet)
        {
            var candidateBox = ToBox(candidatePoly.BoundingBox);

            double usedRight = candidateBox.MaxX;
            double usedTop = candidateBox.MaxY;
            double usedMinX = candidateBox.MinX;
            double usedMinY = candidateBox.MinY;

            double totalPlacedArea = candidatePoly.AbsoluteArea;

            for (int i = 0; i < sheet.Placed.Count; i++)
            {
                var placed = sheet.Placed[i];
                var placedBox = GetPlacedBox(placed);

                usedRight = Math.Max(usedRight, placedBox.MaxX);
                usedTop = Math.Max(usedTop, placedBox.MaxY);
                usedMinX = Math.Min(usedMinX, placedBox.MinX);
                usedMinY = Math.Min(usedMinY, placedBox.MinY);

                totalPlacedArea += placed.Orientation.CanonicalPolygon.AbsoluteArea;
            }

            double usedWidth = Math.Max(0.0, usedRight - Math.Min(0.0, usedMinX));
            double usedHeight = Math.Max(0.0, usedTop - Math.Min(0.0, usedMinY));
            double usedArea = usedWidth * usedHeight;

            double contact =
                ComputeSheetEdgeContact(candidateBox) +
                ComputePartContact(candidateBox, sheet);

            double looseIslandPenalty = 0.0;

            if (sheet.Placed.Count > 0 && contact <= ContactTolerance)
            {
                // Penalize placements that create separated islands instead of
                // packing against existing material or the sheet edge.
                looseIslandPenalty = candidateBox.Width + candidateBox.Height;
            }

            double bottomLeftBias =
                candidateBox.MinY * BottomBiasWeight +
                candidateBox.MinX * LeftBiasWeight;

            double wasteInsideEnvelope = Math.Max(0.0, usedArea - totalPlacedArea);

            double total =
                usedArea * UsedAreaWeight +
                usedTop * UsedTopWeight +
                usedRight * UsedRightWeight +
                bottomLeftBias +
                wasteInsideEnvelope * 2.0 +
                looseIslandPenalty * LooseIslandPenaltyWeight -
                contact * ContactRewardWeight;

            return new PlacementScore
            {
                Total = total,
                UsedRight = usedRight,
                UsedTop = usedTop,
                UsedArea = usedArea,
                ContactLength = contact,
                WasteInsideEnvelope = wasteInsideEnvelope
            };
        }

        private bool IsBetterPlacement(
            PlacementScore score,
            double x,
            double y,
            BestPlacement best)
        {
            if (score.Total < best.Score.Total - 1e-9)
                return true;

            if (Math.Abs(score.Total - best.Score.Total) > 1e-9)
                return false;

            if (score.UsedArea < best.Score.UsedArea - 1e-9)
                return true;

            if (Math.Abs(score.UsedArea - best.Score.UsedArea) > 1e-9)
                return false;

            if (score.UsedTop < best.Score.UsedTop - 1e-9)
                return true;

            if (Math.Abs(score.UsedTop - best.Score.UsedTop) > 1e-9)
                return false;

            if (score.UsedRight < best.Score.UsedRight - 1e-9)
                return true;

            if (Math.Abs(score.UsedRight - best.Score.UsedRight) > 1e-9)
                return false;

            if (score.ContactLength > best.Score.ContactLength + 1e-9)
                return true;

            if (Math.Abs(score.ContactLength - best.Score.ContactLength) > 1e-9)
                return false;

            if (y < best.Y - 1e-9)
                return true;

            if (Math.Abs(y - best.Y) <= 1e-9 && x < best.X - 1e-9)
                return true;

            return false;
        }

        private double ComputeSheetEdgeContact(Box2 box)
        {
            double contact = 0.0;

            double sheetMinX = _request.Margin;
            double sheetMinY = _request.Margin;
            double sheetMaxX = _request.SheetWidth - _request.Margin;
            double sheetMaxY = _request.SheetHeight - _request.Margin;

            if (Near(box.MinX, sheetMinX)) contact += box.Height;
            if (Near(box.MinY, sheetMinY)) contact += box.Width;
            if (Near(box.MaxX, sheetMaxX)) contact += box.Height;
            if (Near(box.MaxY, sheetMaxY)) contact += box.Width;

            return contact;
        }

        private double ComputePartContact(Box2 candidate, SheetState sheet)
        {
            double contact = 0.0;

            for (int i = 0; i < sheet.Placed.Count; i++)
            {
                var placed = GetPlacedBox(sheet.Placed[i]);

                if (Near(candidate.MinX, placed.MaxX))
                {
                    contact += OverlapLength(
                        candidate.MinY,
                        candidate.MaxY,
                        placed.MinY,
                        placed.MaxY);
                }

                if (Near(candidate.MaxX, placed.MinX))
                {
                    contact += OverlapLength(
                        candidate.MinY,
                        candidate.MaxY,
                        placed.MinY,
                        placed.MaxY);
                }

                if (Near(candidate.MinY, placed.MaxY))
                {
                    contact += OverlapLength(
                        candidate.MinX,
                        candidate.MaxX,
                        placed.MinX,
                        placed.MaxX);
                }

                if (Near(candidate.MaxY, placed.MinY))
                {
                    contact += OverlapLength(
                        candidate.MinX,
                        candidate.MaxX,
                        placed.MinX,
                        placed.MaxX);
                }
            }

            return contact;
        }

        private static double OverlapLength(
            double aMin,
            double aMax,
            double bMin,
            double bMax)
        {
            double lo = Math.Max(aMin, bMin);
            double hi = Math.Min(aMax, bMax);
            return Math.Max(0.0, hi - lo);
        }

        private static bool Near(double a, double b)
        {
            return Math.Abs(a - b) <= ContactTolerance;
        }

        private static Box2 ToBox(BoundingBox2D bb)
        {
            return new Box2
            {
                MinX = bb.MinX,
                MinY = bb.MinY,
                MaxX = bb.MaxX,
                MaxY = bb.MaxY
            };
        }

        private static Box2 GetPlacedBox(PlacedItem placed)
        {
            var bb = placed.Orientation.CanonicalPolygon.BoundingBox;

            return new Box2
            {
                MinX = bb.MinX + placed.X,
                MinY = bb.MinY + placed.Y,
                MaxX = bb.MaxX + placed.X,
                MaxY = bb.MaxY + placed.Y
            };
        }

        // ------------------------------------------------------------------
        // Per-call state
        // ------------------------------------------------------------------

        private sealed class SheetState
        {
            public List<PlacedItem> Placed { get; } = new List<PlacedItem>();

            public Dictionary<int, Paths64> ForbiddenByOrientation { get; }
                = new Dictionary<int, Paths64>();

            public SheetState Clone()
            {
                var copy = new SheetState();
                copy.Placed.AddRange(this.Placed);
                return copy;
            }
        }

        private readonly struct PlacedItem
        {
            public OrientedPart Orientation { get; }
            public double X { get; }
            public double Y { get; }

            public PlacedItem(OrientedPart orientation, double x, double y)
            {
                Orientation = orientation;
                X = x;
                Y = y;
            }
        }

        private sealed class BestPlacement
        {
            public double X;
            public double Y;
            public OrientedPart Orientation;
            public PlacementScore Score;
        }

        private struct PlacementScore
        {
            public double Total;
            public double UsedRight;
            public double UsedTop;
            public double UsedArea;
            public double ContactLength;
            public double WasteInsideEnvelope;
        }

        private struct Box2
        {
            public double MinX;
            public double MinY;
            public double MaxX;
            public double MaxY;

            public double Width => MaxX - MinX;
            public double Height => MaxY - MinY;
        }
    }
}
