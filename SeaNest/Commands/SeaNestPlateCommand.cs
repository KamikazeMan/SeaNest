using Rhino;
using Rhino.Commands;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.Geometry;

namespace SeaNest.Commands
{
    public class SeaNestPlateCommand : Command
    {
        public override string EnglishName => "SeaNestPlate";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var settings = SeaNestPlugin.Instance.UserSettings;
            double width = settings.PlateWidth;
            double height = settings.PlateHeight;

            // Ask user to click insertion point
            var gp = new GetPoint();
            gp.SetCommandPrompt("Click to place nesting plate corner");
            gp.Get();

            if (gp.CommandResult() != Result.Success)
                return gp.CommandResult();

            var origin = gp.Point();

            // Ask for plate dimensions
            var gw = new GetNumber();
            gw.SetCommandPrompt($"Plate width (current: {width})");
            gw.SetDefaultNumber(width);
            gw.AcceptNothing(true);
            gw.Get();
            if (gw.CommandResult() == Result.Success)
                width = gw.Number();

            var gh = new GetNumber();
            gh.SetCommandPrompt($"Plate height (current: {height})");
            gh.SetDefaultNumber(height);
            gh.AcceptNothing(true);
            gh.Get();
            if (gh.CommandResult() == Result.Success)
                height = gh.Number();

            // Create the plate rectangle
            var plane = new Plane(origin, Vector3d.ZAxis);
            var rect = new Rectangle3d(plane, width, height);
            var curve = rect.ToNurbsCurve();

            // Add to document on a dedicated layer
            var layerName = "SeaNest_Plates";
            var layerIndex = doc.Layers.FindByFullPath(layerName, -1);
            if (layerIndex < 0)
            {
                var layer = new Rhino.DocObjects.Layer
                {
                    Name = layerName,
                    Color = System.Drawing.Color.FromArgb(0, 206, 201),
                };
                layerIndex = doc.Layers.Add(layer);
            }

            var attr = new Rhino.DocObjects.ObjectAttributes
            {
                LayerIndex = layerIndex,
                Name = $"Plate_{width}x{height}",
            };

            doc.Objects.AddCurve(curve, attr);

            // Save updated settings
            settings.PlateWidth = width;
            settings.PlateHeight = height;
            settings.Save();

            RhinoApp.WriteLine($"SeaNest: Plate created — {width}\" x {height}\" at ({origin.X:F2}, {origin.Y:F2}, {origin.Z:F2})");

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
