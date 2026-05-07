using Rhino;
using Rhino.Commands;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.DocObjects;

namespace SeaNest.Commands
{
    public class SeaNestLabelsCommand : Command
    {
        public override string EnglishName => "SeaNestLabels";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var settings = SeaNestPlugin.Instance.UserSettings;
            double height = settings.LabelHeight;
            string prefix = settings.LabelPrefix;

            var go = new GetObject();
            go.SetCommandPrompt("Select surfaces to label");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go.EnablePreSelect(true, true);
            go.GetMultiple(1, 0);

            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            // Ask for label height
            var gh = new GetNumber();
            gh.SetCommandPrompt($"Label text height (current: {height})");
            gh.SetDefaultNumber(height);
            gh.AcceptNothing(true);
            gh.Get();

            if (gh.CommandResult() == Result.Success)
                height = gh.Number();

            settings.LabelHeight = height;
            settings.Save();

            RhinoApp.WriteLine($"SeaNest: Labeling {go.ObjectCount} parts — prefix: \"{prefix}\", height: {height}\"...");

            // TODO Phase 2: Implement part labeling
            // - Get or generate part name from object name/layer
            // - Find center point of each surface
            // - Create text geometry at center
            // - Project/engrave text onto surface
            // - Support auto-numbering with prefix

            RhinoApp.WriteLine("SeaNest: Part labeling — geometry engine coming in Phase 2.");

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
