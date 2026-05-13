using Rhino.Geometry;

namespace SeaNest.Geometry
{
    /// <summary>
    /// Phase 20a — output of the rat-holes joint-cutter compute. Contains
    /// the accumulated cutter Breps for the frame (compound: rectangle slot
    /// + full-circle rat-hole) and the stringer (single stadium slot),
    /// plus the joint geometry for diagnostics and downstream consumers.
    ///
    /// Phase 20a.8: moved into the shared Geometry namespace. Used by
    /// SeaNestRatHolesCommand only; SeaNestSlotsCommand returns its
    /// member cutters directly (single Brep[]) and doesn't need the
    /// FrameCutters slot.
    /// </summary>
    public class JointCutResult
    {
        public Brep[] FrameCutters;
        public Brep[] StringerCutters;
        public Point3d FrameBottomAnchor;
        public Point3d StringerTopAnchor;
        public Point3d StringerMidPoint;
        public Vector3d UpDir;
        public Vector3d FrameNormal;
        public Vector3d StringerNormal;
    }
}
