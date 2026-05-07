using System;
using System.IO;
using Rhino;
using Newtonsoft.Json;

namespace SeaNest
{
    public class SeaNestSettings
    {
        // === Slot Settings ===
        public double SlotWidth { get; set; } = 0.25;
        public double SlotDepthRatio { get; set; } = 0.5;

        // === Rat Hole Settings ===
        public double RatHoleRadius { get; set; } = 0.125;

        // === Label Settings ===
        public double LabelHeight { get; set; } = 0.5;
        public string LabelPrefix { get; set; } = "P";

        // === Frame Line Settings ===
        public bool ShowCenterlines { get; set; } = true;
        public bool ShowStationMarks { get; set; } = true;

        // === Nesting Settings ===
        public double NestSpacing { get; set; } = 1.0;
        public double PlateWidth { get; set; } = 48.0;
        public double PlateHeight { get; set; } = 96.0;
        public double SimplificationFactor { get; set; } = 100.0;

        // === Trim Settings ===
        public double TrimTolerance { get; set; } = 0.001;

        // === General ===
        public string UnitSystem { get; set; } = "Auto";
        public int AutosaveIntervalMinutes { get; set; } = 5;
        public bool SoundEnabled { get; set; } = true;

        // === File Path ===
        [JsonIgnore]
        private static string SettingsPath
        {
            get
            {
                var folder = Path.Combine(
                    Environment.GetFolderPath(Environment.SpecialFolder.ApplicationData),
                    "SeaNest");
                Directory.CreateDirectory(folder);
                return Path.Combine(folder, "settings.json");
            }
        }

        public static SeaNestSettings Load()
        {
            try
            {
                if (File.Exists(SettingsPath))
                {
                    var json = File.ReadAllText(SettingsPath);
                    var settings = JsonConvert.DeserializeObject<SeaNestSettings>(json);
                    if (settings != null)
                    {
                        RhinoApp.WriteLine("SeaNest: Settings loaded.");
                        return settings;
                    }
                }
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"SeaNest: Could not load settings — {ex.Message}");
            }

            return new SeaNestSettings();
        }

        public void Save()
        {
            try
            {
                var json = JsonConvert.SerializeObject(this, Formatting.Indented);
                File.WriteAllText(SettingsPath, json);
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"SeaNest: Could not save settings — {ex.Message}");
            }
        }
    }
}
