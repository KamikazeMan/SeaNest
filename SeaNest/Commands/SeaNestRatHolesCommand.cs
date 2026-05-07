using System;
using System.Collections.Generic;
using System.Linq;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Geometry.Intersect;
using Rhino.Input;
using Rhino.Input.Custom;

namespace SeaNest.Commands
{
    public class SeaNestRatHolesCommand : Command
    {
        public override string EnglishName => "SeaNestRatHoles";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            try { return RunInner(doc); }
            catch (Exception ex) { RhinoApp.WriteLine("Crash: " + ex.Message); return Result.Failure; }
        }

        private Result RunInner(RhinoDoc doc)
        {
            bool useInches = doc.ModelUnitSystem == Rhino.UnitSystem.Inches ||
                             doc.ModelUnitSystem == Rhino.UnitSystem.Feet;
            string unitLabel = useInches ? "in" : "mm";

            // --- Rat hole radius ---
            double radius = useInches ? 0.5 : 12.5;
            var gw = new GetNumber();
            gw.SetCommandPrompt(string.Format("Rat hole radius ({0})", unitLabel));
            gw.SetDefaultNumber(radius);
            gw.SetLowerLimit(0.001, false);
            gw.AcceptNothing(true);
            if (gw.Get() == GetResult.Number) radius = gw.Number();

            // --- Slot option ---
            var gs = new GetOption();
            gs.SetCommandPrompt("Add keyhole slot above rat hole?");
            int optSlotYes = gs.AddOption("Yes");
            int optSlotNo = gs.AddOption("No");
            gs.SetDefaultString("No");
            gs.AcceptNothing(true);
            bool addSlot = false;
            if (gs.Get() == GetResult.Option && gs.Option().Index == optSlotYes)
                addSlot = true;

            double slotWidth = 0;
            double slotLength = 0;
            if (addSlot)
            {
                slotWidth = useInches ? 0.5 : 12.5;
                var gsw = new GetNumber();
                gsw.SetCommandPrompt(string.Format("Slot width ({0})", unitLabel));
                gsw.SetDefaultNumber(slotWidth);
                gsw.SetLowerLimit(0.001, false);
                gsw.AcceptNothing(true);
                if (gsw.Get() == GetResult.Number) slotWidth = gsw.Number();

                slotLength = useInches ? 2.0 : 50.0;
                var gsl = new GetNumber();
                gsl.SetCommandPrompt(string.Format("Slot length ({0})", unitLabel));
                gsl.SetDefaultNumber(slotLength);
                gsl.SetLowerLimit(0.001, false);
                gsl.AcceptNothing(true);
                if (gsl.Get() == GetResult.Number) slotLength = gsl.Number();
            }

            // --- Debug markers ---
            var gd = new GetOption();
            gd.SetCommandPrompt("Show debug markers at each hole position?");
            int optYes = gd.AddOption("Yes");
            int optNo = gd.AddOption("No");
            gd.SetDefaultString("No");
            gd.AcceptNothing(true);
            bool showDebug = false;
            if (gd.Get() == GetResult.Option && gd.Option().Index == optYes)
                showDebug = true;

            RhinoApp.WriteLine(string.Format("SeaNest Rat Holes: radius {0} {1}, slot={2}",
                radius, unitLabel, addSlot ? string.Format("{0}x{1}", slotWidth, slotLength) : "no"));

            // Select frames
            var go = new GetObject();
            go.SetCommandPrompt("Select FRAMES (plates that will get rat holes)");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go.EnablePreSelect(false, false);
            go.GetMultiple(1, 0);
            if (go.CommandResult() != Result.Success) return go.CommandResult();

            var frames = new List<Brep>();
            var frameIds = new List<Guid>();
            for (int i = 0; i < go.ObjectCount; i++)
            {
                var b = go.Object(i).Brep();
                if (b != null && b.IsValid)
                {
                    frames.Add(b.DuplicateBrep());
                    frameIds.Add(go.Object(i).ObjectId);
                }
            }

            // Select stringers
            var go2 = new GetObject();
            go2.SetCommandPrompt("Now select STRINGERS");
            go2.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go2.EnablePreSelect(false, false);
            go2.GetMultiple(1, 0);
            if (go2.CommandResult() != Result.Success) return go2.CommandResult();

            var stringers = new List<Brep>();
            for (int i = 0; i < go2.ObjectCount; i++)
            {
                var b = go2.Object(i).Brep();
                if (b != null && b.IsValid) stringers.Add(b.DuplicateBrep());
            }

            if (frames.Count == 0 || stringers.Count == 0)
            {
                RhinoApp.WriteLine("Need at least 1 frame and 1 stringer.");
                return Result.Failure;
            }

            int layerIdx = GetOrCreateLayer(doc, "SeaNest_RatHoles", System.Drawing.Color.FromArgb(243, 156, 18));
            int debugLayerIdx = GetOrCreateLayer(doc, "SeaNest_DebugMarkers", System.Drawing.Color.Red);

            int totalHoles = 0;
            int successful = 0;

            for (int f = 0; f < frames.Count; f++)
            {
                var frame = frames[f];

                // Find frame's main face
                BrepFace mainFace = null;
                double maxArea = 0;
                foreach (var face in frame.Faces)
                {
                    try
                    {
                        var fb = face.DuplicateFace(false);
                        var amp = AreaMassProperties.Compute(fb);
                        if (amp != null && amp.Area > maxArea)
                        {
                            maxArea = amp.Area;
                            mainFace = face;
                        }
                    }
                    catch { }
                }

                if (mainFace == null) continue;

                var plateNormal = mainFace.NormalAt(mainFace.Domain(0).Mid, mainFace.Domain(1).Mid);
                plateNormal.Unitize();

                var frameFaceBrep = mainFace.DuplicateFace(false);

                Plane facePlane;
                bool isPlanar = mainFace.TryGetPlane(out facePlane, doc.ModelAbsoluteTolerance);
                if (!isPlanar)
                {
                    var midU = mainFace.Domain(0).Mid;
                    var midV = mainFace.Domain(1).Mid;
                    var midPt = mainFace.PointAt(midU, midV);
                    facePlane = new Plane(midPt, plateNormal);
                }

                Vector3d faceDown = -Vector3d.ZAxis;
                faceDown = faceDown - (faceDown * facePlane.Normal) * facePlane.Normal;
                if (!faceDown.Unitize()) faceDown = -facePlane.YAxis;

                var bbox = frame.GetBoundingBox(true);
                double thickness = Math.Min(Math.Min(bbox.Diagonal.X, bbox.Diagonal.Y), bbox.Diagonal.Z);

                // Holes: store both the point and the slot direction (along stringer, downhill)
                var holeData = new List<Tuple<Point3d, Vector3d>>();
                double dedupeTol = Math.Max(doc.ModelAbsoluteTolerance * 5.0, radius * 0.5);

                foreach (var stringer in stringers)
                {
                    try
                    {
                        if (Intersection.BrepBrep(frameFaceBrep, stringer, doc.ModelAbsoluteTolerance,
                            out Curve[] curves, out Point3d[] pts))
                        {
                            if (curves == null || curves.Length == 0) continue;

                            var joined = Curve.JoinCurves(curves, doc.ModelAbsoluteTolerance);
                            var loops = (joined != null && joined.Length > 0) ? joined : curves;

                            foreach (var loop in loops)
                            {
                                if (loop == null || !loop.IsValid) continue;

                                int samples = 200;
                                var sampledPoints = new List<Point3d>();
                                for (int k = 0; k <= samples; k++)
                                {
                                    if (!loop.NormalizedLengthParameter((double)k / samples, out double t))
                                        continue;
                                    sampledPoints.Add(loop.PointAt(t));
                                }

                                if (sampledPoints.Count < 2) continue;

                                // Find stringer direction (two farthest points)
                                double maxDist = 0;
                                Point3d slotA = sampledPoints[0];
                                Point3d slotB = sampledPoints[0];
                                for (int i = 0; i < sampledPoints.Count; i++)
                                {
                                    for (int j = i + 1; j < sampledPoints.Count; j++)
                                    {
                                        double d = sampledPoints[i].DistanceTo(sampledPoints[j]);
                                        if (d > maxDist)
                                        {
                                            maxDist = d;
                                            slotA = sampledPoints[i];
                                            slotB = sampledPoints[j];
                                        }
                                    }
                                }

                                Vector3d slotDir = slotB - slotA;
                                if (!slotDir.Unitize()) continue;
                                if (slotDir * faceDown < 0) slotDir = -slotDir;

                                Point3d centroid;
                                try
                                {
                                    var amp = AreaMassProperties.Compute(loop);
                                    centroid = amp != null ? amp.Centroid : loop.GetBoundingBox(true).Center;
                                }
                                catch { centroid = loop.GetBoundingBox(true).Center; }

                                double maxDir = double.NegativeInfinity;
                                foreach (var pt in sampledPoints)
                                {
                                    double score = (pt - centroid) * slotDir;
                                    if (score > maxDir) maxDir = score;
                                }

                                // Find points near lower end and average them (better centering)
                                var lowerEndPoints = sampledPoints
                                    .Where(p => Math.Abs((p - centroid) * slotDir - maxDir) < radius * 0.5)
                                    .ToList();
                                if (lowerEndPoints.Count == 0) lowerEndPoints = sampledPoints;

                                Point3d holePt = new Point3d(0, 0, 0);
                                foreach (var pt in lowerEndPoints)
                                    holePt += (Vector3d)pt;
                                holePt /= lowerEndPoints.Count;

                                if (!mainFace.ClosestPoint(holePt, out double u, out double v))
                                    continue;
                                var onFace = mainFace.PointAt(u, v);

                                bool isDuplicate = holeData.Any(p => p.Item1.DistanceTo(onFace) <= dedupeTol);
                                if (!isDuplicate)
                                    holeData.Add(Tuple.Create(onFace, slotDir));
                            }
                        }
                    }
                    catch (Exception ex) { RhinoApp.WriteLine("  Intersection error: " + ex.Message); }
                }

                if (holeData.Count == 0)
                {
                    RhinoApp.WriteLine(string.Format("  Frame {0}: no stringer intersections", f + 1));
                    continue;
                }

                RhinoApp.WriteLine(string.Format("  Frame {0}: {1} unique rat hole(s)", f + 1, holeData.Count));

                if (showDebug)
                {
                    var debugAttr = new ObjectAttributes { LayerIndex = debugLayerIdx };
                    foreach (var entry in holeData)
                        doc.Objects.AddPoint(entry.Item1, debugAttr);
                }

                double bigDepth = thickness * 20;
                var current = frame.DuplicateBrep();
                int cutsMade = 0;

                for (int c = 0; c < holeData.Count; c++)
                {
                    var onFace = holeData[c].Item1;
                    var slotDir = holeData[c].Item2;
                    bool success = false;

                    Vector3d localNormal = plateNormal;
                    if (mainFace.ClosestPoint(onFace, out double hu, out double hv))
                    {
                        localNormal = mainFace.NormalAt(hu, hv);
                        localNormal.Unitize();
                    }

                    // Attempt 1: try to build combined rat hole + slot cutter
                    success = TryCutWithSlot(ref current, onFace, localNormal, slotDir,
                        radius, bigDepth, addSlot, slotWidth, slotLength, doc.ModelAbsoluteTolerance);

                    // Attempt 2: shift slightly along normal
                    if (!success)
                    {
                        var shiftedPt = onFace + localNormal * (thickness * 0.1);
                        success = TryCutWithSlot(ref current, shiftedPt, localNormal, slotDir,
                            radius, bigDepth, addSlot, slotWidth, slotLength, doc.ModelAbsoluteTolerance);
                    }

                    // Attempt 3: slightly larger radius
                    if (!success)
                    {
                        success = TryCutWithSlot(ref current, onFace, localNormal, slotDir,
                            radius * 1.05, bigDepth, addSlot, slotWidth, slotLength, doc.ModelAbsoluteTolerance);
                    }

                    if (success)
                    {
                        cutsMade++;
                        RhinoApp.WriteLine(string.Format("    Hole {0}: OK", c + 1));
                    }
                    else
                    {
                        RhinoApp.WriteLine(string.Format("    Hole {0}: FAILED after 3 attempts", c + 1));
                    }
                }

                if (cutsMade > 0)
                {
                    var attr = new ObjectAttributes { LayerIndex = layerIdx };
                    doc.Objects.AddBrep(current, attr);
                    doc.Objects.Delete(frameIds[f], true);
                    successful++;
                    totalHoles += cutsMade;
                }
            }

            RhinoApp.WriteLine("==========================================");
            RhinoApp.WriteLine(string.Format("  Rat Holes: {0} frames, {1} holes", successful, totalHoles));
            RhinoApp.WriteLine("==========================================");

            doc.Views.Redraw();
            return Result.Success;
        }

        // Cut rat hole (cylinder) + optional keyhole slot (rectangular prism going up)
        private bool TryCutWithSlot(ref Brep current, Point3d center, Vector3d normal, Vector3d slotDir,
            double radius, double depth, bool addSlot, double slotWidth, double slotLength, double tolerance)
        {
            try
            {
                var cutters = new List<Brep>();

                // 1. Rat hole: cylindrical cutter
                var circlePlane = new Plane(center - normal * depth / 2, normal);
                var circle = new Circle(circlePlane, radius);
                var cyl = new Cylinder(circle, depth);
                var cylBrep = cyl.ToBrep(true, true);

                if (cylBrep == null || !cylBrep.IsValid) return false;
                cutters.Add(cylBrep);

                // 2. Keyhole slot: rectangular prism extending UP from center (opposite slotDir)
                if (addSlot && slotLength > 0 && slotWidth > 0)
                {
                    // Project world +Z onto the face to get "up along stringer" direction
                    Vector3d upDir = Vector3d.ZAxis;
                    upDir = upDir - (upDir * normal) * normal;
                    if (!upDir.Unitize())
                    {
                        // Fallback: use slotDir reversed
                        upDir = -slotDir;
                        if (!upDir.Unitize())
                        {
                            RhinoApp.WriteLine("    Slot: upDir failed");
                            return false;
                        }
                    }

                    Vector3d widthAxis = Vector3d.CrossProduct(normal, upDir);
                    if (!widthAxis.Unitize())
                    {
                        RhinoApp.WriteLine("    Slot: widthAxis failed");
                        return false;
                    }

                    

                    double halfW = slotWidth / 2.0;
                    var slotPlane = new Plane(center, widthAxis, upDir);

                    var slotRect = new Rectangle3d(slotPlane, new Interval(-halfW, halfW),
                        new Interval(-radius * 0.5, slotLength));
                    var slotCurve = slotRect.ToNurbsCurve();

                    if (slotCurve == null)
                    {
                        RhinoApp.WriteLine("    Slot: curve build failed");
                    }
                    else
                    {
                        // The curve's plane normal = +widthAxis × upDir. For our plane that's -normal (world).
                        // So we need to translate OPPOSITE to the extrusion direction to center it.
                        // Translate by +normal * depth/2 so extrusion passes through center.
                        var movedCurve = slotCurve.DuplicateCurve();
                        movedCurve.Translate(normal * depth / 2);

                        var slotExt = Extrusion.Create(movedCurve, depth, true);
                        if (slotExt == null)
                        {
                            RhinoApp.WriteLine("    Slot: extrusion failed");
                        }
                        else
                        {
                            var slotBrep = slotExt.ToBrep();
                            if (slotBrep == null || !slotBrep.IsValid)
                            {
                                RhinoApp.WriteLine("    Slot: invalid brep");
                            }
                            else
                            {
                                

                              cutters.Add(slotBrep);
                                
                            }
                        }
                    }
                }

                // Cut all cutters at once - Rhino handles overlapping cutters better in one pass
                double oldV = GetVolume(current);

                var diffAll = Brep.CreateBooleanDifference(
                    new[] { current },
                    cutters.ToArray(),
                    tolerance,
                    false);

                Brep result = null;
                if (diffAll != null && diffAll.Length > 0)
                    result = diffAll.Where(b => b != null && b.IsValid)
                        .OrderByDescending(b => GetVolume(b)).FirstOrDefault();

                if (result != null && GetVolume(result) < oldV * 0.999)
                {
                    current = result;
                    return true;
                }

                // Fallback: cut hole, then slot, one at a time
                RhinoApp.WriteLine("    Falling back to sequential cuts");

                var diff1 = Brep.CreateBooleanDifference(
                    new[] { current },
                    new[] { cutters[0] },
                    tolerance,
                    false);

                Brep afterHole = null;
                if (diff1 != null && diff1.Length > 0)
                    afterHole = diff1.Where(b => b != null && b.IsValid)
                        .OrderByDescending(b => GetVolume(b)).FirstOrDefault();

                if (afterHole == null || GetVolume(afterHole) >= oldV * 0.999)
                    return false;

                if (cutters.Count == 1)
                {
                    current = afterHole;
                    return true;
                }

                // Cut slot from the result with larger tolerance
                double afterHoleV = GetVolume(afterHole);
                var diff2 = Brep.CreateBooleanDifference(
                    new[] { afterHole },
                    new[] { cutters[1] },
                    tolerance * 10,
                    false);

                Brep afterSlot = null;
                if (diff2 != null && diff2.Length > 0)
                    afterSlot = diff2.Where(b => b != null && b.IsValid)
                        .OrderByDescending(b => GetVolume(b)).FirstOrDefault();

                if (afterSlot != null && GetVolume(afterSlot) < afterHoleV * 0.999)
                {
                    current = afterSlot;
                    return true;
                }

                current = afterHole;
                RhinoApp.WriteLine("    Slot cut failed, kept rat hole only");
                return true;
            }
            catch { }

            return false;
        }

        private int GetOrCreateLayer(RhinoDoc doc, string name, System.Drawing.Color color)
        {
            int idx = doc.Layers.FindByFullPath(name, -1);
            if (idx < 0) { var l = new Layer { Name = name, Color = color }; idx = doc.Layers.Add(l); }
            return idx;
        }

        private double GetVolume(Brep brep)
        {
            try { var vp = VolumeMassProperties.Compute(brep); if (vp != null) return vp.Volume; } catch { }
            try { return brep.GetBoundingBox(true).Diagonal.Length; } catch { }
            return 0;
        }
    }
}