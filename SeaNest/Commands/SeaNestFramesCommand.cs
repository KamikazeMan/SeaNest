using Rhino;
using Rhino.Commands;
using Rhino.Input.Custom;
using Rhino.DocObjects;

namespace SeaNest.Commands
{
    public class SeaNestFramesCommand : Command
    {
        public override string EnglishName => "SeaNestFrames";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var settings = SeaNestPlugin.Instance.UserSettings;

            var go = new GetObject();
            go.SetCommandPrompt("Select surfaces for frame lines");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go.EnablePreSelect(true, true);
            go.GetMultiple(1, 0);

            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            RhinoApp.WriteLine($"SeaNest: Adding frame lines to {go.ObjectCount} surfaces...");
            RhinoApp.WriteLine($"  Centerlines: {(settings.ShowCenterlines ? "ON" : "OFF")}");
            RhinoApp.WriteLine($"  Station marks: {(settings.ShowStationMarks ? "ON" : "OFF")}");

            // TODO Phase 2: Implement frame lines
            // - Calculate centerline of each surface
            // - Draw centerline as curve on dedicated layer
            // - Add station marks at regular intervals
            // - Support user-defined station spacing

            RhinoApp.WriteLine("SeaNest: Frame lines — geometry engine coming in Phase 2.");

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
