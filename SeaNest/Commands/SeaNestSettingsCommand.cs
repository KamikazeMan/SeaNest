using Rhino;
using Rhino.Commands;
using Rhino.Input;
using Rhino.Input.Custom;

namespace SeaNest.Commands
{
    public class SeaNestSettingsCommand : Command
    {
        public override string EnglishName => "SeaNestSettings";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var settings = SeaNestPlugin.Instance.UserSettings;

            RhinoApp.WriteLine("=== SeaNest Settings ===");
            RhinoApp.WriteLine($"  Slot Width:       {settings.SlotWidth}\"");
            RhinoApp.WriteLine($"  Rat Hole Radius:  {settings.RatHoleRadius}\"");
            RhinoApp.WriteLine($"  Label Height:     {settings.LabelHeight}\"");
            RhinoApp.WriteLine($"  Label Prefix:     \"{settings.LabelPrefix}\"");
            RhinoApp.WriteLine($"  Nest Spacing:     {settings.NestSpacing}\"");
            RhinoApp.WriteLine($"  Plate Size:       {settings.PlateWidth}\" x {settings.PlateHeight}\"");
            RhinoApp.WriteLine($"  Trim Tolerance:   {settings.TrimTolerance}");
            RhinoApp.WriteLine($"  Unit System:      {settings.UnitSystem}");
            RhinoApp.WriteLine($"  Autosave:         Every {settings.AutosaveIntervalMinutes} min");

            // Interactive setting modification
            var gs = new GetString();
            gs.SetCommandPrompt("Setting to change (or Enter to close)");
            gs.AcceptNothing(true);
            gs.SetDefaultString("");
            gs.AddOption("SlotWidth");
            gs.AddOption("RatHoleRadius");
            gs.AddOption("LabelHeight");
            gs.AddOption("LabelPrefix");
            gs.AddOption("NestSpacing");
            gs.AddOption("PlateWidth");
            gs.AddOption("PlateHeight");
            gs.AddOption("TrimTolerance");
            gs.AddOption("Units");
            gs.AddOption("Autosave");
            gs.AddOption("ResetAll");

            var result = gs.Get();
            if (result == GetResult.Option)
            {
                var option = gs.Option().EnglishName;
                ModifySetting(settings, option);
            }

            return Result.Success;
        }

        private void ModifySetting(SeaNestSettings settings, string option)
        {
            switch (option)
            {
                case "SlotWidth":
                    settings.SlotWidth = AskNumber("Slot width", settings.SlotWidth);
                    break;
                case "RatHoleRadius":
                    settings.RatHoleRadius = AskNumber("Rat hole radius", settings.RatHoleRadius);
                    break;
                case "LabelHeight":
                    settings.LabelHeight = AskNumber("Label height", settings.LabelHeight);
                    break;
                case "LabelPrefix":
                    var gp = new GetString();
                    gp.SetCommandPrompt($"Label prefix (current: \"{settings.LabelPrefix}\")");
                    gp.SetDefaultString(settings.LabelPrefix);
                    if (gp.Get() == GetResult.String)
                        settings.LabelPrefix = gp.StringResult();
                    break;
                case "NestSpacing":
                    settings.NestSpacing = AskNumber("Nest spacing", settings.NestSpacing);
                    break;
                case "PlateWidth":
                    settings.PlateWidth = AskNumber("Plate width", settings.PlateWidth);
                    break;
                case "PlateHeight":
                    settings.PlateHeight = AskNumber("Plate height", settings.PlateHeight);
                    break;
                case "TrimTolerance":
                    settings.TrimTolerance = AskNumber("Trim tolerance", settings.TrimTolerance);
                    break;
                case "Units":
                    var gu = new GetString();
                    gu.SetCommandPrompt("Unit system");
                    gu.AddOption("Auto");
                    gu.AddOption("Inches");
                    gu.AddOption("Millimeters");
                    gu.AddOption("Feet");
                    if (gu.Get() == GetResult.Option)
                        settings.UnitSystem = gu.Option().EnglishName;
                    break;
                case "Autosave":
                    settings.AutosaveIntervalMinutes = (int)AskNumber("Autosave interval (minutes)", settings.AutosaveIntervalMinutes);
                    break;
                case "ResetAll":
                    RhinoApp.WriteLine("SeaNest: Settings reset to defaults.");
                    var fresh = new SeaNestSettings();
                    SeaNestPlugin.Instance.GetType().GetProperty("UserSettings").SetValue(SeaNestPlugin.Instance, fresh);
                    fresh.Save();
                    return;
            }

            settings.Save();
            RhinoApp.WriteLine("SeaNest: Setting updated and saved.");
        }

        private double AskNumber(string prompt, double current)
        {
            var gn = new GetNumber();
            gn.SetCommandPrompt($"{prompt} (current: {current})");
            gn.SetDefaultNumber(current);
            gn.AcceptNothing(true);
            if (gn.Get() == GetResult.Number)
                return gn.Number();
            return current;
        }
    }
}
