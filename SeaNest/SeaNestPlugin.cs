using Rhino;
using Rhino.PlugIns;
using Rhino.UI;
using System;

namespace SeaNest
{
    public class SeaNestPlugin : PlugIn
    {
        public static SeaNestPlugin Instance { get; private set; }
        public SeaNestSettings UserSettings { get; private set; }

        public SeaNestPlugin()
        {
            Instance = this;
        }

        protected override LoadReturnCode OnLoad(ref string errorMessage)
        {
            UserSettings = SeaNestSettings.Load();

            var panelType = typeof(SeaNestPanel);
            Panels.RegisterPanel(this, panelType, "SeaNest", null);

            RhinoApp.WriteLine("SeaNest v1.0 loaded — Precision Automation for Marine Builders");
            return LoadReturnCode.Success;
        }

        protected override void OnShutdown()
        {
            UserSettings?.Save();
            base.OnShutdown();
        }
    }
}