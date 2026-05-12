namespace SeaNest
{
    // Phase 17 — shared Rhino layer names used by SeaNestNest (writes) and
    // SeaNestExport (reads). Kept as a single source of truth so the export
    // command cannot drift from the layer scheme the nesting command emits.
    internal static class SeaNestLayers
    {
        public const string Nested = "SeaNest_Nested";
        public const string Sheets = "SeaNest_Sheets";
        public const string Labels = "SeaNest_Labels";
        public const string Scribe = "SeaNest_Scribe";
    }
}
