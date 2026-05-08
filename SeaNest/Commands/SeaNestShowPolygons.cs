using System.Linq;
using Rhino;
using Rhino.Commands;
using Rhino.DocObjects;
using Rhino.Geometry;
using Rhino.Input.Custom;
using SeaNest.Nesting.Core.Geometry;
using SeaNest.RhinoAdapters;

namespace SeaNest.Commands
{
    public class SeaNestShowPolygons : Command
    {
        public override string EnglishName => "SeaNestShowPolygons";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var go = new GetObject();
            go.SetCommandPrompt("Select a frame to inspect");
            go.GeometryFilter = ObjectType.Brep | ObjectType.Extrusion;
            go.SubObjectSelect = false;
            go.Get();
            if (go.CommandResult() != Result.Success) return go.CommandResult();

            var brep = go.Object(0).Brep();
            if (brep == null)
            {
                var ext = go.Object(0).Geometry() as Extrusion;
                if (ext != null) brep = ext.ToBrep();
            }
            if (brep == null) return Result.Failure;

            var flat = BrepFlattener.Flatten(brep, doc);
            if (flat == null)
            {
                RhinoApp.WriteLine("Could not flatten.");
                return Result.Failure;
            }
            var poly = flat.Outer;

            // Draw the polygon as a closed polyline with each vertex as a point object.
            var polyline = new Polyline(poly.Count + 1);
            foreach (var pt in poly.Points) polyline.Add(pt.X, pt.Y, 0);
            polyline.Add(poly.Points[0].X, poly.Points[0].Y, 0);

            doc.Objects.AddPolyline(polyline);
            foreach (var pt in poly.Points)
                doc.Objects.AddPoint(new Point3d(pt.X, pt.Y, 0));

            doc.Views.Redraw();
            RhinoApp.WriteLine($"Drew {poly.Count}-vertex polygon with vertex markers.");
            return Result.Success;
        }
    }
}