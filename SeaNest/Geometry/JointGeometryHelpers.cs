using System;
using System.Collections.Generic;
using System.Linq;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;

namespace SeaNest.Geometry
{
    /// <summary>
    /// Phase 20a.8 — shared joint-geometry helpers extracted from
    /// SeaNestRatHolesCommand for reuse by SeaNestSlotsCommand (Phase 20b)
    /// and any future joint-cut command. Pure functions over Brep/curve
    /// geometry — no Rhino doc dependency, no command-specific constants.
    ///
    /// The algorithm primitives this class provides:
    ///   - <see cref="GetPlateInfo"/> finds a thin plate's mid-plane via
    ///     Phase 14.1.2's twin-pair detection, with a largest-planar +
    ///     bbox-extent fallback for plates where twin-pair fails.
    ///   - <see cref="SectionPlateAtMidPlane"/> intersects a plate Brep
    ///     with its mid-plane to produce the centerline outline.
    ///   - <see cref="FindExtremeJointHit"/> intersects the joint
    ///     centerline (= mid-plane × mid-plane line) with each plate's
    ///     centerline outline; returns the top- or bottom-most hit.
    ///   - <see cref="BuildStadiumSlotCutter"/> builds the generic slot
    ///     cutter shape (stadium with chord at Y=−L, dome past chord at
    ///     Y=−(L+r)) extruded perpendicular to the plate face.
    ///   - <see cref="ExtrudeClosedPlanarCurve"/> +
    ///     <see cref="EnsureOutwardOrientation"/> produce closed-manifold
    ///     cutters with correct face normals.
    ///   - <see cref="ApplyCutters"/> performs the boolean difference with
    ///     a sequential-fallback path that survives single-cutter failures.
    /// </summary>
    public static class JointGeometryHelpers
    {
        // ---------------------------------------------------------------
        // Plate mid-plane detection
        // ---------------------------------------------------------------

        /// <summary>
        /// Identify a thin plate's mid-plane.
        ///
        /// Primary path uses <c>BrepFlattener.FindTwinPlateFace</c> —
        /// Phase 14.1.2's twin-pair detection that's tuned for marine
        /// plate geometry (10° anti-parallel tolerance, 0.7 area-ratio
        /// minimum, 0.1 separation-ratio maximum). The returned plane is
        /// the mid-plane (origin = midpoint of the two face centroids,
        /// normal = averaged unit normals); thickness is derived as
        /// 2 × perpendicular distance from the returned face's centroid
        /// to the mid-plane.
        ///
        /// Fallback path (when twin-pair fails): uses the largest planar
        /// face and synthesizes a mid-plane by projecting every brep
        /// vertex onto that face's normal — thickness = max-projection −
        /// min-projection, mid-plane origin at the midpoint of that
        /// range.
        /// </summary>
        public static PlateInfo GetPlateInfo(Brep brep, double tol)
        {
            // Primary: Phase 14.1.2 twin-pair detection via BrepFlattener.
            var twin = SeaNest.RhinoAdapters.BrepFlattener.FindTwinPlateFace(brep, tol);
            if (twin != null)
            {
                var face = twin.Value.Face;
                var midPlane = twin.Value.Plane;
                Vector3d normal = midPlane.Normal;
                if (!normal.Unitize()) throw new Exception("twin-pair mid-plane normal degenerate");

                double thickness;
                using (var faceBrep = face.DuplicateFace(false))
                {
                    var amp = AreaMassProperties.Compute(faceBrep);
                    if (amp == null) throw new Exception("twin-pair face centroid failed");
                    double signedDist = (amp.Centroid - midPlane.Origin) * normal;
                    thickness = 2.0 * Math.Abs(signedDist);
                }
                if (thickness < tol)
                    throw new Exception("twin-pair thickness ≤ tolerance");

                return new PlateInfo
                {
                    MidPlane = midPlane,
                    Normal = normal,
                    Thickness = thickness,
                };
            }

            // Fallback: largest planar face + bbox extents along its normal.
            BrepFace largest = null;
            Plane largestPlane = Plane.Unset;
            double maxArea = 0;
            foreach (var face in brep.Faces)
            {
                if (!face.IsPlanar(tol * 10.0)) continue;
                if (!face.TryGetPlane(out Plane p, tol * 10.0)) continue;
                try
                {
                    using (var fb = face.DuplicateFace(false))
                    {
                        var amp = AreaMassProperties.Compute(fb);
                        if (amp != null && amp.Area > maxArea)
                        {
                            maxArea = amp.Area;
                            largest = face;
                            largestPlane = p;
                        }
                    }
                }
                catch { }
            }
            if (largest == null)
                throw new Exception("no twin pair and no planar face — not a recognizable plate");

            Vector3d fallbackNormal = largestPlane.Normal;
            if (!fallbackNormal.Unitize()) throw new Exception("largest-face normal degenerate");

            double minProj = double.PositiveInfinity;
            double maxProj = double.NegativeInfinity;
            foreach (var v in brep.Vertices)
            {
                var d = v.Location - largestPlane.Origin;
                double proj = d.X * fallbackNormal.X + d.Y * fallbackNormal.Y + d.Z * fallbackNormal.Z;
                if (proj < minProj) minProj = proj;
                if (proj > maxProj) maxProj = proj;
            }
            double fallbackThickness = maxProj - minProj;
            if (fallbackThickness < tol)
                throw new Exception("plate thickness ≤ tolerance (fallback path)");

            double midProj = (minProj + maxProj) * 0.5;
            var fallbackMidOrigin = largestPlane.Origin + fallbackNormal * midProj;
            var fallbackMidPlane = new Plane(fallbackMidOrigin, fallbackNormal);

            return new PlateInfo
            {
                MidPlane = fallbackMidPlane,
                Normal = fallbackNormal,
                Thickness = fallbackThickness,
            };
        }

        // ---------------------------------------------------------------
        // Joint centerline + anchor finding
        // ---------------------------------------------------------------

        /// <summary>
        /// Section a plate Brep with its mid-plane via
        /// <c>Intersection.BrepPlane</c>. Returns the centerline outline
        /// curves (typically one closed curve for a simple plate, possibly
        /// more for plates with holes). Empty array on failure.
        /// </summary>
        public static Curve[] SectionPlateAtMidPlane(Brep brep, Plane midPlane, double tol)
        {
            if (!Intersection.BrepPlane(brep, midPlane, tol, out Curve[] curves, out _))
                return Array.Empty<Curve>();
            return curves ?? Array.Empty<Curve>();
        }

        /// <summary>
        /// Intersect the joint line with each outline curve; return the
        /// topmost or bottommost hit (along <paramref name="upDir"/>).
        /// Joint line is extended to cover each outline's bbox so we
        /// don't miss hits because the input <see cref="Line"/> happens
        /// to be a short segment.
        /// </summary>
        public static Point3d FindExtremeJointHit(
            Curve[] outline, Line jointLine, Vector3d upDir, bool findTop, double tol)
        {
            var bbox = BoundingBox.Empty;
            foreach (var c in outline) if (c != null) bbox.Union(c.GetBoundingBox(true));
            double extend = bbox.Diagonal.Length + 1.0;
            var extPt0 = jointLine.From - jointLine.Direction * extend;
            var extPt1 = jointLine.To + jointLine.Direction * extend;
            var extLineCurve = new LineCurve(extPt0, extPt1);

            bool any = false;
            double bestScore = findTop ? double.NegativeInfinity : double.PositiveInfinity;
            Point3d bestHit = Point3d.Origin;

            foreach (var curve in outline)
            {
                if (curve == null) continue;
                var ci = Intersection.CurveCurve(curve, extLineCurve, tol, tol);
                if (ci == null) continue;
                foreach (var evt in ci)
                {
                    var pt = evt.PointA;
                    double score = pt.X * upDir.X + pt.Y * upDir.Y + pt.Z * upDir.Z;
                    if (findTop)
                    {
                        if (score > bestScore) { bestScore = score; bestHit = pt; any = true; }
                    }
                    else
                    {
                        if (score < bestScore) { bestScore = score; bestHit = pt; any = true; }
                    }
                }
            }

            if (!any) throw new Exception(
                findTop ? "joint line missed plate outline (top)" : "joint line missed plate outline (bottom)");
            return bestHit;
        }

        /// <summary>
        /// Absolute distance along <paramref name="dir"/> from
        /// <paramref name="a"/> to <paramref name="b"/>.
        /// Direction-agnostic (always positive).
        /// </summary>
        public static double DistanceAlong(Point3d a, Point3d b, Vector3d dir)
        {
            return Math.Abs((b.X - a.X) * dir.X + (b.Y - a.Y) * dir.Y + (b.Z - a.Z) * dir.Z);
        }

        // ---------------------------------------------------------------
        // Cutter geometry
        // ---------------------------------------------------------------

        /// <summary>
        /// Build a stadium-shaped slot cutter (rectangle with rounded
        /// bottom past the chord) extruded perpendicular to the member's
        /// face. The chord ends at Y=−L on the slot plane; the dome
        /// extends past the chord to Y=−(L+r), where r = slotWidth/2.
        /// The top edge of the rectangle overcuts UP past the member's
        /// edge by cutterDepth so the slot opens cleanly at the top.
        ///
        /// Cutter is extruded along <paramref name="memberNormal"/> by
        /// <paramref name="cutterDepth"/> and centered on the member's
        /// mid-plane via <see cref="ExtrudeClosedPlanarCurve"/>.
        ///
        /// Used by both SeaNestRatHolesCommand (member-side slot for the
        /// stringer in a dual-cut joint) and SeaNestSlotsCommand
        /// (member-only slot for a watertight joint where the plate
        /// stays solid).
        /// </summary>
        public static Brep[] BuildStadiumSlotCutter(
            Point3d anchor,
            Vector3d memberNormal,
            Vector3d memberWidthDir,
            Vector3d upDir,
            double slotWidth,
            double slotDepth,
            double cutterDepth,
            double tol)
        {
            double r = slotWidth / 2.0;
            double L = slotDepth;

            var slotPlane = MakeSafeCutPlane(anchor, memberWidthDir, upDir);

            //   p0 = (-r, +cutterDepth)  top-left, above member's top edge (overcut)
            //   p1 = (-r, -L)            left chord endpoint at joint center
            //   arcMid = (0, -(L + r))   apex of half-circle BEYOND the chord
            //   p2 = (+r, -L)            right chord endpoint at joint center
            //   p3 = (+r, +cutterDepth)  top-right, above member's top edge
            //   close p3 → p0            top edge (above member)
            var p0 = slotPlane.PointAt(-r, +cutterDepth);
            var p1 = slotPlane.PointAt(-r, -L);
            var arcMid = slotPlane.PointAt(0, -(L + r));
            var p2 = slotPlane.PointAt(+r, -L);
            var p3 = slotPlane.PointAt(+r, +cutterDepth);

            var leftSide = new LineCurve(p0, p1);
            var bottomArc = new ArcCurve(new Arc(p1, arcMid, p2));
            if (!bottomArc.IsValid) return Array.Empty<Brep>();
            var rightSide = new LineCurve(p2, p3);
            var top = new LineCurve(p3, p0);

            var poly = new PolyCurve();
            poly.Append(leftSide);
            poly.Append(bottomArc);
            poly.Append(rightSide);
            poly.Append(top);
            if (!poly.MakeClosed(0.001)) return Array.Empty<Brep>();

            var cutter = ExtrudeClosedPlanarCurve(poly, memberNormal, cutterDepth);
            return cutter != null ? new[] { cutter } : Array.Empty<Brep>();
        }

        /// <summary>
        /// Build a plane with explicit X/Y axes (right-handed; normal =
        /// xAxis × yAxis). Used so cutter helpers don't depend on
        /// curve-winding-derived plane normals (which can flip extrusion
        /// direction unpredictably).
        /// </summary>
        public static Plane MakeSafeCutPlane(Point3d origin, Vector3d xAxis, Vector3d yAxis)
        {
            var x = xAxis;
            if (!x.Unitize()) x = Vector3d.XAxis;
            var y = yAxis;
            if (!y.Unitize()) y = Vector3d.YAxis;
            return new Plane(origin, x, y);
        }

        /// <summary>
        /// Extrude a closed planar curve perpendicular to its plane along
        /// <paramref name="extrudeDir"/> by <paramref name="depth"/>, cap
        /// the result so it's a closed manifold, then translate by
        /// −extrudeDir × depth/2 so the resulting Brep is centered on the
        /// original curve's plane.
        ///
        /// Uses <c>Surface.CreateExtrusion</c> (direction-explicit,
        /// winding-agnostic) rather than <c>Extrusion.Create</c> which
        /// derives direction from curve winding. <c>CapPlanarHoles</c>
        /// closes the two end planes.
        /// <see cref="EnsureOutwardOrientation"/> flips the result if
        /// face normals point inward.
        /// </summary>
        public static Brep ExtrudeClosedPlanarCurve(Curve profile, Vector3d extrudeDir, double depth)
        {
            if (profile == null) return null;
            var dir = extrudeDir;
            if (!dir.Unitize()) return null;
            var srf = Surface.CreateExtrusion(profile, dir * depth);
            if (srf == null) return null;
            var brep = srf.ToBrep();
            if (brep == null) return null;
            brep = brep.CapPlanarHoles(0.001);
            if (brep == null || !brep.IsValid) return null;
            brep.Translate(dir * (-depth / 2.0));
            EnsureOutwardOrientation(brep);
            return brep;
        }

        /// <summary>
        /// Verify the cutter Brep has outward-pointing face normals and
        /// <c>Flip()</c> if it doesn't.
        ///
        /// Rhino's <c>CreateBooleanDifference</c> treats an INWARD-facing
        /// cutter as describing the void OUTSIDE the cutter's volume —
        /// equivalent to subtracting "everything not the cutter" from the
        /// part. Result: only the cutter-intersect-part sliver survives.
        /// Calling this ensures cutters always read as "solid material to
        /// subtract" regardless of how they were constructed.
        /// </summary>
        public static void EnsureOutwardOrientation(Brep brep)
        {
            if (brep == null) return;
            try
            {
                if (brep.SolidOrientation == BrepSolidOrientation.Inward)
                    brep.Flip();
            }
            catch { }
        }

        // ---------------------------------------------------------------
        // Boolean application
        // ---------------------------------------------------------------

        /// <summary>
        /// Apply a list of accumulated cutters to a single part. Tries
        /// one multi-cutter <c>Brep.CreateBooleanDifference</c> first
        /// (fast path when geometry is well-behaved); on failure falls
        /// back to sequential single-cutter cuts so a single bad cutter
        /// doesn't take the whole batch down. Returns the largest-volume
        /// valid result, or null if every approach fails.
        ///
        /// Preconditions (enforced by the caller's silent-skip): cutters
        /// is non-null and Count &gt; 0.
        /// </summary>
        public static Brep ApplyCutters(Brep part, IReadOnlyList<Brep> cutters, double tol)
        {
            if (part == null || cutters == null || cutters.Count == 0) return null;

            try
            {
                var multi = Brep.CreateBooleanDifference(
                    new[] { part }, cutters.ToArray(), tol, false);
                if (multi != null && multi.Length > 0)
                {
                    var best = multi.Where(b => b != null && b.IsValid)
                                    .OrderByDescending(GetVolume)
                                    .FirstOrDefault();
                    if (best != null) return best;
                }
            }
            catch { }

            // Sequential fallback — silently apply cutters one at a time.
            Brep current = part.DuplicateBrep();
            int seqOk = 0;
            foreach (var cutter in cutters)
            {
                if (cutter == null) continue;
                try
                {
                    var seq = Brep.CreateBooleanDifference(
                        new[] { current }, new[] { cutter }, tol, false);
                    if (seq == null || seq.Length == 0) continue;
                    var seqBest = seq.Where(b => b != null && b.IsValid)
                                     .OrderByDescending(GetVolume)
                                     .FirstOrDefault();
                    if (seqBest == null) continue;
                    current = seqBest;
                    seqOk++;
                }
                catch { }
            }
            return seqOk > 0 ? current : null;
        }

        /// <summary>
        /// Volume of a Brep; falls back to bbox diagonal length when
        /// VolumeMassProperties fails (e.g., for non-closed Breps).
        /// Used by <see cref="ApplyCutters"/> to pick the largest-volume
        /// result from a boolean operation.
        /// </summary>
        public static double GetVolume(Brep brep)
        {
            try { var vp = VolumeMassProperties.Compute(brep); if (vp != null) return vp.Volume; } catch { }
            try { return brep.GetBoundingBox(true).Diagonal.Length; } catch { }
            return 0;
        }
    }
}
