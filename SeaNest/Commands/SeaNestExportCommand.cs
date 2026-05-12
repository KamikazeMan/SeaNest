using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Eto.Forms;
using Rhino;
using Rhino.DocObjects;
using Rhino.FileIO;
using Rhino.UI;
using RhinoCommand = Rhino.Commands.Command;
using Result = Rhino.Commands.Result;
using RunMode = Rhino.Commands.RunMode;

namespace SeaNest.Commands
{
    // Phase 17 — export the nested output (SeaNest_Nested curves +
    // SeaNest_Labels text) to a DXF/DWG file for plate cutters.
    //
    // Sheet outlines (SeaNest_Sheets) are intentionally excluded: operators
    // know sheet size from the nesting prompt and do not need it in the cut
    // file. If that changes, add SeaNestLayers.Sheets to LayerNames below.
    //
    // Strategy: pre-select the objects on the target layers, then call
    // RhinoDoc.WriteFile with WriteSelectedObjectsOnly + SuppressDialogBoxes.
    // This avoids mangling layer visibility and reuses Rhino's last-saved
    // DXF scheme. Prior selection is restored in a finally block so the
    // user's document state is preserved across success, cancel, and error.
    public class SeaNestExportCommand : RhinoCommand
    {
        public override string EnglishName => "SeaNestExport";

        private static readonly string[] LayerNames =
        {
            SeaNestLayers.Nested,
            SeaNestLayers.Labels,
        };

        protected override Result RunCommand(RhinoDoc doc, RunMode mode)
        {
            var layerIndices = new List<int>();
            foreach (var name in LayerNames)
            {
                int idx = doc.Layers.FindByFullPath(name, -1);
                if (idx >= 0) layerIndices.Add(idx);
            }

            if (layerIndices.Count == 0)
            {
                RhinoApp.WriteLine(
                    "SeaNestExport: no SeaNest output layers found. " +
                    "Run SeaNestNest first to produce nested geometry.");
                return Result.Nothing;
            }

            var layerIndexSet = new HashSet<int>(layerIndices);
            var exportable = new List<RhinoObject>();
            int curveCount = 0;
            int textCount = 0;
            foreach (var ro in doc.Objects)
            {
                if (ro == null || ro.IsDeleted) continue;
                if (!layerIndexSet.Contains(ro.Attributes.LayerIndex)) continue;
                exportable.Add(ro);
                if (ro.ObjectType == ObjectType.Annotation) textCount++;
                else curveCount++;
            }

            if (exportable.Count == 0)
            {
                RhinoApp.WriteLine(
                    "SeaNestExport: SeaNest layers exist but contain no geometry. " +
                    "Run SeaNestNest first to produce nested output.");
                return Result.Nothing;
            }

            string defaultName = string.IsNullOrEmpty(doc.Name)
                ? "Nested.dxf"
                : Path.GetFileNameWithoutExtension(doc.Name) + "_Nested.dxf";

            string targetPath;
            using (var dlg = new SaveFileDialog())
            {
                dlg.Title = "Export nested output";
                dlg.FileName = defaultName;
                dlg.Filters.Add(new FileFilter("AutoCAD DXF (*.dxf)", ".dxf"));
                dlg.Filters.Add(new FileFilter("AutoCAD DWG (*.dwg)", ".dwg"));
                dlg.CurrentFilterIndex = 0;

                if (dlg.ShowDialog(RhinoEtoApp.MainWindow) != DialogResult.Ok)
                    return Result.Cancel;

                targetPath = dlg.FileName;
            }

            if (string.IsNullOrEmpty(targetPath))
                return Result.Cancel;

            string ext = Path.GetExtension(targetPath);
            if (string.IsNullOrEmpty(ext) ||
                (!ext.Equals(".dxf", StringComparison.OrdinalIgnoreCase) &&
                 !ext.Equals(".dwg", StringComparison.OrdinalIgnoreCase)))
            {
                targetPath += ".dxf";
            }

            var previouslySelected = doc.Objects
                .GetSelectedObjects(includeLights: true, includeGrips: false)
                .Select(o => o.Id)
                .ToList();

            bool ok = false;
            try
            {
                doc.Objects.UnselectAll();
                foreach (var ro in exportable)
                    doc.Objects.Select(ro.Id, true);

                var opts = new FileWriteOptions
                {
                    WriteSelectedObjectsOnly = true,
                    SuppressDialogBoxes = true,
                };

                ok = doc.WriteFile(targetPath, opts);
            }
            catch (Exception ex)
            {
                RhinoApp.WriteLine($"SeaNestExport: write failed — {ex.Message}");
                ok = false;
            }
            finally
            {
                doc.Objects.UnselectAll();
                foreach (var id in previouslySelected)
                    doc.Objects.Select(id, true);
            }

            if (!ok)
            {
                RhinoApp.WriteLine($"SeaNestExport: failed to write {targetPath}.");
                return Result.Failure;
            }

            long sizeBytes = 0;
            try { sizeBytes = new FileInfo(targetPath).Length; }
            catch { }
            double sizeKb = sizeBytes / 1024.0;

            RhinoApp.WriteLine(
                $"SeaNestExport: wrote {curveCount} curve(s) + {textCount} label(s) " +
                $"to {targetPath} ({sizeKb:0.0} KB).");

            return Result.Success;
        }
    }
}
