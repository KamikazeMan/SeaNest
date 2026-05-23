using System;
using System.Collections.Generic;
using System.IO;
using SeaNest.Nesting.Core.Geometry;

namespace SeaNest.Nesting.Core.Nesting
{
    public enum PartClass
    {
        Strip,
        Irregular
    }

    public static class PartClassifier
    {
        public const double StripMinAspectRatio = 8.0;
        public const double StripMaxShortSide = 15.0;
        public const double StripMaxConcavityFraction = 0.10;

        private const string OutputPath =
            @"C:\Users\David LaChute\Desktop\phase24a_classify.txt";

        public static PartClass Classify(Polygon polygon)
        {
            var bbox = polygon.BoundingBox;
            double w = bbox.Width;
            double h = bbox.Height;
            double longSide = Math.Max(w, h);
            double shortSide = Math.Min(w, h);
            double aspect = longSide / Math.Max(shortSide, 1e-6);

            double bboxArea = w * h;
            double polyArea = polygon.AbsoluteArea;
            double concavityFraction = bboxArea > 1e-12
                ? 1.0 - (polyArea / bboxArea)
                : 0.0;

            if (aspect >= StripMinAspectRatio &&
                shortSide <= StripMaxShortSide &&
                concavityFraction <= StripMaxConcavityFraction)
            {
                return PartClass.Strip;
            }
            return PartClass.Irregular;
        }

        public static void RunDiagnostic(
            IReadOnlyList<Polygon> polygons,
            Action<string> log)
        {
            if (polygons == null || log == null) return;

            var lines = new List<string>();
            lines.Add($"Phase 24a classification run at {DateTime.Now:yyyy-MM-dd HH:mm:ss}");
            lines.Add($"Thresholds: aspect>={StripMinAspectRatio}, shortSide<={StripMaxShortSide}\", concavity<={StripMaxConcavityFraction:P0}");
            lines.Add("");

            int stripCount = 0;
            int irregularCount = 0;

            for (int i = 0; i < polygons.Count; i++)
            {
                var poly = polygons[i];
                var cls = Classify(poly);

                var bbox = poly.BoundingBox;
                double w = bbox.Width;
                double h = bbox.Height;
                double longSide = Math.Max(w, h);
                double shortSide = Math.Min(w, h);
                double aspect = longSide / Math.Max(shortSide, 1e-6);
                double bboxArea = w * h;
                double polyArea = poly.AbsoluteArea;
                double concavity = bboxArea > 1e-12 ? 1.0 - (polyArea / bboxArea) : 0.0;

                lines.Add(
                    $"  Part {i}: class={cls}, " +
                    $"bbox={w:F2}x{h:F2}, " +
                    $"area={polyArea:F2}, " +
                    $"aspect={aspect:F2}, " +
                    $"concavity={concavity:P1}, " +
                    $"verts={poly.Count}");

                if (cls == PartClass.Strip) stripCount++;
                else irregularCount++;
            }

            lines.Add("");
            lines.Add($"Phase 24a aggregate: {stripCount} strips, {irregularCount} irregulars out of {polygons.Count} parts.");

            try
            {
                File.WriteAllLines(OutputPath, lines);
                log($"Phase 24a classification complete — see Desktop\\phase24a_classify.txt ({stripCount} strips, {irregularCount} irregulars)");
            }
            catch (Exception ex)
            {
                log($"Phase 24a classification: file write failed: {ex.Message}");
                foreach (var line in lines)
                    log(line);
            }
        }
    }
}
