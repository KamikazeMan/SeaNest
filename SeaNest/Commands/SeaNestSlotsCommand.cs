using Rhino;
using Rhino.Commands;
using Rhino.Input;
using Rhino.Input.Custom;
using Rhino.DocObjects;

namespace SeaNest.Commands
{
    public class SeaNestSlotsCommand : Command
    {
        public override string EnglishName => "SeaNestSlots";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var settings = SeaNestPlugin.Instance.UserSettings;

            // Get slot width from settings or ask user
            double slotWidth = settings.SlotWidth;

            // Step 1: Select surfaces to add slots to
            var go = new GetObject();
            go.SetCommandPrompt("Select surfaces for interlocking slots");
            go.GeometryFilter = ObjectType.Surface | ObjectType.Brep;
            go.EnablePreSelect(true, true);
            go.GetMultiple(2, 0);

            if (go.CommandResult() != Result.Success)
                return go.CommandResult();

            // Step 2: Get slot width option
            var gw = new GetNumber();
            gw.SetCommandPrompt($"Slot width (current: {slotWidth})");
            gw.SetDefaultNumber(slotWidth);
            gw.AcceptNothing(true);
            gw.Get();

            if (gw.CommandResult() == Result.Success)
                slotWidth = gw.Number();

            // Save updated setting
            settings.SlotWidth = slotWidth;
            settings.Save();

            RhinoApp.WriteLine($"SeaNest: Creating interlocking slots — width: {slotWidth}\" on {go.ObjectCount} surfaces...");

            // TODO Phase 3: Implement actual slot cutting geometry
            // - Find intersection curves between selected surfaces
            // - Create slot notches at intersection points
            // - Boolean difference to cut slots
            // - Support preview before applying

            RhinoApp.WriteLine("SeaNest: Slot creation — geometry engine coming in Phase 3.");

            doc.Views.Redraw();
            return Result.Success;
        }
    }
}
