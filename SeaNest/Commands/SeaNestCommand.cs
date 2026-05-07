using Rhino;
using Rhino.Commands;
using Rhino.UI;

namespace SeaNest.Commands
{
    public class SeaNestCommand : Command
    {
        public override string EnglishName => "SeaNest";

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var panelId = SeaNestPanel.PanelId;
            var visible = Panels.IsPanelVisible(panelId);

            if (visible)
            {
                Panels.ClosePanel(panelId);
            }
            else
            {
                Panels.OpenPanel(panelId);
            }

            return Result.Success;
        }
    }
}
