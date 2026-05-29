# Phase 30 Design — Coordinated Frame Nester (audit + design)

**Status:** design only, no implementation. Branch: `claude/phase29-cpsat-spike`.
**Decision carried in:** CP-SAT is the wrong tool (valid-but-loose layouts;
coupling-bound super-linear solve; rotations explode the model). Pivot to a
custom sequential coordinated-frame placement heuristic that emits the large
frames as `PreplacedPart`s for the existing engine to fill around.

> All signatures below were read from the code at HEAD of this branch and are
> cited `file:line`. Open questions for David/you are flagged **[OPEN]**.

---

## PART 1 — AUDIT

### A. PreplacedPart / PlaceAllWithPreplaced (Phase 24b)

`NfpPlacementEngine.cs:136`
```csharp
public PreplacedPart(
    int originalIndex, int sheet, OrientedPart orientation,
    double x, double y, PlacementResult placement)
```
`NfpPlacementEngine.cs:149`
```csharp
public NestResult PlaceAllWithPreplaced(
    IReadOnlyList<int> partOrder,
    IReadOnlyList<PreplacedPart> preplaced)
```

**How it works** (`:161-217`): for each pre-placed part it seeds the sheet with
`new PlacedItem(pp.Orientation, pp.X, pp.Y)` (`:166`), appends `pp.Placement` to
the output list (`:169`), then **clears every sheet's forbidden-region cache**
(`:174-175`) so the first real placement rebuilds NFPs against the full
pre-placed state. It then places `partOrder` normally via `TryPlaceOnSheet`,
spilling to new sheets if needed.

**Dual role (important):** `Orientation/X/Y` drive *collision/forbidden-region*
geometry; `Placement` is *only* the output record that gets returned. The two
must agree or the output will disagree with the collision model.

**Can it take 5 frames + nest ~26 around them?** Yes — no count limit; it just
seeds `Placed` and fills the rest. Limitations:
- **Single-sheet intent is not enforced** — if a remaining part can't fit it
  spills to sheet 2 (`:194-207`). The frame nester must place frames such that
  the rest genuinely fit, or accept multi-sheet.
- **Pre-placed parts are never moved** — fill is greedy around fixed frames.
- **Orientation indices must not collide** — the existing reference code assigns
  fresh indices past all existing ones (`:927-931`); the frame nester must do
  the same (see H).

**Working reference template:** `NestingEngine.cs:933-1025` already does exactly
this for StripMajor/Irregular → then `PlaceAllWithPreplaced(gapFillIndices,
preplacedList)` for StripMinor+Tiny (`:1025`). The frame nester is a new
"preplaced producer" that slots in *ahead* of this same pattern.

### B. NoFitPolygon.Compute + orientation system

`NoFitPolygon.cs:164`
```csharp
public static IReadOnlyList<Polygon> Compute(
    Polygon a, Polygon b, double spacing,
    int srcOrientationIndex = -1, int candOrientationIndex = -1)
```
Takes **Polygons** (not OrientedParts). Returns the NFP as a list of `Polygon`
contours; outer = CCW (`Area>0`), holes = CW. Convention (validated in spikes,
`NfpPlacementEngine.cs:665`): with A at its coords, B's reference **translation**
is forbidden iff inside the NFP; placing B's ref **on the NFP boundary** = B
touching A with `spacing` clearance.

`NfpCache.cs:31` — preferred entry point (memoized by orientation-index pair):
```csharp
public IReadOnlyList<Polygon> Get(OrientedPart a, OrientedPart b)
```
Internally calls `Compute(a.CanonicalPolygon, b.CanonicalPolygon, _spacing, …)`.

**Orientations** (`OrientedPart.cs`):
- `OrientedPart.Build(int orientationIndex, int sourcePartIndex, Polygon source, double rotationDeg, bool isMirrored)` (`:94`).
- `BuildAll(...)` (`:147-189`) enumerates `rotationCount = round(360/step)`
  rotations × (mirror? 2:1), assigning **globally-unique** `OrientationIndex`
  sequentially (original-then-mirror). For 90°+mirror = **8 orientations/part**.
- `CanonicalPolygon`: mirror → rotate about origin → translate so **bbox-min is
  at (0,0)** → normalized CCW. `BBox` is cached, min at origin.
- Properties: `OrientationIndex, SourcePartIndex, RotationDeg, IsMirrored,
  CanonicalPolygon, BBox`.

**Output representation** — `PlacementResult.cs:109`:
```csharp
public PlacementResult(
    int originalIndex, int sheet, Transform2D transform,
    double rotationDeg, bool isMirrored,
    double sourceBBoxMinX, double sourceBBoxMaxX, Polygon placedPolygon)
```
Canonical transform-construction chain (`NestingEngine.cs:947-951`, identical in
`NfpPlacementEngine` TryPlace/beam paths):
```csharp
step1 = Translation(-srcBBox.MinX, -srcBBox.MinY)
step2 = RotationDegrees(rotDeg)
step3 = Translation(-rotBBox.MinX, -rotBBox.MinY)
step4 = Translation(px, py)
combined = step1.Then(step2).Then(step3).Then(step4)
placedPoly = rotatedNormalized.Translate(px, py)   // == CanonicalPolygon.Translate(x,y)
```

### C. PartClassification (Phase 24a) — identifying the frames

`PartClassification.cs:29`
```csharp
public static PartClass Classify(Polygon polygon)   // {StripMajor, StripMinor, Irregular, Tiny}
```
Thresholds (`:18-24`): Tiny if `longSide<=2.0`; Strip if `aspect>=8.0 &&
shortSide<=15.0 && concavity<=maxConcavity` (StripMajor if `longSide>=50.0`,
else StripMinor); else **Irregular**. `concavity = 1 - polyArea/bboxArea`.

**The frame set already exists, computed at `NestingEngine.cs:280-291`:**
```csharp
double criticalThreshold = sheetArea * 0.05;          // 5% of sheet
// Irregular parts with bbox area >= criticalThreshold  ->  engine.CriticalPartIndices
```
So **the large hull frames == `CriticalPartIndices`** (Irregular + bbox ≥ 5%
sheet). `IrregularPartIndices` (`:294-300`) is the broader Irregular set. This is
exactly the programmatic frame selector we need — no new threshold required,
though it's reusable/tunable. **[OPEN: is "Irregular + bbox≥5% sheet" exactly the
5 frames on the real job, or does it also catch a 6th large irregular? Confirm on
the benchmark.]**

### D. Existing contact / scoring helpers — and a key limitation

`NfpPlacementEngine.cs`:
- `ScorePlacementCandidate(Polygon candidatePoly, SheetState sheet) → PlacementScore` (`:1559`).
- `PlacementScore` struct (`:1819`): `Total, UsedRight, UsedTop, UsedArea, ContactLength, WasteInsideEnvelope`.
- Weights (`:279-285`): UsedArea 100, UsedTop 25, UsedRight 5, BottomBias 0.50, LeftBias 0.10, **ContactReward 30**, LooseIslandPenalty 4.
- `ComputeSheetEdgeContact(Box2 box)` (`:1673`) and `ComputePartContact(Box2 candidate, SheetState sheet)` (`:1690`).
- Enumeration: `FindCandidateVertices(Paths64 feasible, int maxCount)` (`:754`, dedups + sorts lower-left); `SampleInteriorPoints(Paths64, double step)` (`:829`); `EnumerateFeasibleRegions(int,SheetState)` (`:924`); `MeasureFeasibleArea(int,SheetState)` (`:1039`); `CountValidPlacements(int,SheetState,int)` (`:950`).
- `OverlapChecker.Overlaps(Polygon, Polygon, tolerance)`.

**⚠️ Critical finding:** `ComputePartContact`/`ComputeSheetEdgeContact` operate on
`Box2` (axis-aligned **bounding boxes**), measuring bbox-edge adjacency — **not
true polygon-edge contact.** For frames whose concave edge must *cradle* the next
frame's convex back, bbox-adjacency is the wrong signal (two interlocking frames
have heavily overlapping bboxes and little bbox-edge contact). **The frame nester
needs a new polygon-profile contact/gap metric** (see F). This is the main "build
new" item the audit surfaces.

---

## PART 2 — DESIGN

### E. Frame ordering (sheer-line sequence)

**Recommend: order frames by centroid projection onto the sheet's long (X) axis,
ascending; tiebreak by centroid Y, then `SourcePartIndex`.** Rationale: the human
stacks along the hull sheer line, which runs the length of the sheet; centroid-X
order reproduces that sweep and makes each frame nest against the immediately
prior one. `Polygon.Centroid` exists (area-weighted). Using original index is
fragile (depends on selection/model order); size order doesn't follow the sheer.
**[OPEN: is the sheer axis always the sheet long axis, or should we PCA the frame
centroids to find the true sheer direction? For a 240×72 sheet, X is almost
certainly right, but PCA is a cheap robustness upgrade.]**

### F. Placing one frame against the stack

**Candidate generation — use the NFP boundary (this is the whole trick).**
For the incoming frame F (each orientation) against an already-placed frame:
1. `nfp = NfpCache.Get(placedOrient, Fconvert)` — but note cache builds B-rel-A;
   we want F's translation locus, so compute against each placed frame and union
   the forbidden regions exactly as `BuildForbiddenRegionSerial` does
   (`:653-668`: `nfpPoly.Translate(placed.X, placed.Y)` then `Clipper.Union`).
2. Feasible region = IFP(sheet) − unionForbidden (reuse `ComputeFeasibleRegion`
   pattern). Candidate positions = **`FindCandidateVertices(feasible, maxCount)`**
   — the vertices of the feasible region are exactly the NFP-boundary contact
   points, i.e. positions where F touches the stack with `spacing` clearance.
   This inherently yields "mated" candidates; no interior sampling needed for the
   tight nest (interior points sit *away* from contact). Keep `SampleInteriorPoints`
   only as a fallback if boundary candidates all fail.

**"Against the union of placed frames" vs "most-recent only":** **Recommend
union of all placed frames** for correctness (F must not overlap *any* placed
frame, and can contact two at once — a frame cradled between two neighbors).
Most-recent-only is cheaper but loses the multi-contact nests the human achieves.
The union is what `BuildForbiddenRegion*` already produces, so it's free.

**Tightest-nest metric (new, polygon-based).** For each candidate
(orientation, position) that passes `OverlapChecker.Overlaps == false` against
all placed frames, score (lower = better):
```
score =  w_bbox  * (Δ half-perimeter of the combined frame-stack bbox)   // growth
       - w_contact * profileContactLength(F_placed, placedFrames)         // true edge contact
       + w_gap    * gapPocketArea(F_placed, placedFrames)                 // trapped voids
```
- `Δ half-perimeter`: cheap, linear-ish, penalizes spread in both axes (the
  fix for CP-SAT's fan; we learned width-only is too weak).
- `profileContactLength`: **new helper** — sum of near-coincident polygon-edge
  overlap between F and each placed frame (walk edges, accumulate length where
  edge-to-edge distance < `ContactTolerance`). This is the concave-cradle signal
  the existing `Box2` contact can't express.
- `gapPocketArea`: optional — area of bounded voids created between F and the
  stack (discourages leaving unfillable pockets). Could approximate via
  `MeasureFeasibleArea`-style Clipper area of the trapped region; **[OPEN: is gap
  penalty worth the cost, or does contact-maximization already avoid pockets?]**
- **[OPEN: weights `w_*`. Start `w_contact` dominant (mirror ContactReward≈30 vs
  area≈100 relationship), tune on the benchmark. Same "sweepable env var" trick
  as RegretWeight if we want to tune without rebuilds.]**

**Commit:** pick the min-score candidate, re-validate with
`OverlapChecker.Overlaps` against all placed frames (belt-and-suspenders), append
to the placed-frame list, proceed to next frame.

### G. Failure handling (graceful degradation)

Per frame, in order, stop at first that holds:
1. No non-overlapping candidate in any orientation on the current sheet →
2. retry with `SampleInteriorPoints` fallback (denser positions) →
3. relax: drop the tightness objective, accept any valid on-sheet placement
   (bottom-left, like the base engine) →
4. still nothing → **leave the frame for the normal engine**: do *not* pre-place
   it; add it to the `partOrder` handed to `PlaceAllWithPreplaced` so the beam/
   greedy path tries (and may legitimately spill to a 2nd sheet).

Never force overlap; never silently drop. Log each degradation step (matches the
existing diagnostic-file convention). **[OPEN: is a 2nd sheet acceptable for
frame-heavy jobs, or is single-sheet a hard requirement that should fail loudly?]**

### H. Integration into the pipeline

The frame nester is a **preplaced producer** that runs *before* the existing
Phase 24b fill, reusing its exact emit pattern (`NestingEngine.cs:933-1010`):
1. Frames = `CriticalPartIndices` (C). Order them (E). Place them (F/G) →
   produce `List<PreplacedPart>` with fresh orientation indices
   (`nextOrientIdx++`, per `:927-931`) and the step1–step4 transform/PlacementResult.
2. Hand **all remaining parts** (everything not pre-placed) as `partOrder` to
   `PlaceAllWithPreplaced(remaining, framePreplaced)` — the existing engine fills
   stringers/strips/tiny/small-irregulars around the frame stack.
3. Position flow: frame `(orientation, x, y)` → `PlacedItem` collision +
   `PlacementResult` output, exactly as A describes.

**Placement in the mode hierarchy — recommend a new selectable nesting mode**
("Coordinated frames"), not a silent replacement: the beam path is fine for
non-frame jobs, and we want A/B comparison. **[OPEN: auto-trigger when
`CriticalPartIndices.Count >= N` (e.g. ≥3 large frames), or always user-selected?
Recommend: user-selectable now, consider auto-trigger after it beats the beam on
the benchmark.]** It can run *instead of* the Phase 27 beam retry for frame-heavy
jobs, or *before* it as a seed.

### I. Determinism (must preserve byte-identical output)

We fought hard for this (Phase 28.1.5 total-order sorts). The frame nester must:
- **Deterministic frame order** (E): centroid-X with explicit total-order
  tiebreaks (centroid-Y, then `SourcePartIndex`) — never rely on hash/set order.
- **Ordered candidate enumeration:** `FindCandidateVertices` already sorts
  lower-left deterministically; iterate orientations in ascending
  `OrientationIndex`.
- **Total-order tiebreak on score ties:** reuse the `ComparePlacementSequence`
  philosophy (`NfpPlacementEngine.cs:1440`) — on equal score, break by
  (orientationIndex, then x, then y) so two equal candidates pick the same one
  every run.
- **No parallelism** in the frame loop (it's tiny; ~5 frames). NFP cache is
  already single-flight/deterministic.
- Validate by the existing harness: run benchmark twice at seed 0, diff excluding
  timestamp — must be byte-identical.

### J. Implementation plan (testable increments)

Each increment independently runnable/diffable on the 31-part benchmark.

1. **Frame selection + ordering + emit, fixed orientation, most-recent contact.**
   Select `CriticalPartIndices`, order by centroid-X, place each at its
   single best NFP-boundary vertex against the *most-recent* frame only (no
   rotation search, simple max-contact), emit `PreplacedPart`s, call
   `PlaceAllWithPreplaced`. *Test:* all frames placed, no overlaps
   (`OverlapChecker`), output deterministic. Likely still loose — that's fine.
2. **Add rotation search.** Loop all orientations per frame, pick best
   (orientation, position). *Test:* tighter than #1; still valid + deterministic.
3. **Add multi-frame (union) contact + the polygon `profileContactLength`
   metric + half-perimeter growth term.** *Test:* visually interlocking stack
   (not a fan); compare combined-bbox area vs #2 and vs the human reference.
4. **Wire full fill + degradation (G) + mode selection (H).** *Test:* full
   31-part nest on 1 sheet (target), determinism diff clean, utilization vs the
   beam's 8-frames-max ceiling.
5. **(Stretch) tuning:** sweepable weights; optional gap-pocket penalty;
   Hertel–Mehlhorn-free since we're not in CP-SAT anymore.

---

## Consolidated open questions for David / you
1. **Frame set:** confirm `Irregular + bbox≥5% sheet` == exactly the 5 frames on
   the real job (E/C). PCA the sheer axis or trust sheet-X (E)?
2. **Metric weights** and whether to include the gap-pocket term (F).
3. **Single-sheet hard requirement** or allow 2nd-sheet spill (G)?
4. **Mode:** user-selected vs auto-trigger on frame count (H)?
5. **Contact definition:** is "edges within `ContactTolerance`" the right
   profile-contact measure, or do we want true shared-boundary length via a
   Clipper intersection of slightly-inflated F against the stack? (F — the new
   helper is the riskiest new geometry; worth agreeing before increment 3.)

**No code written.** Recommend approving increments 1–2 to start, with the
profile-contact metric (3) gated on resolving open question #5.
