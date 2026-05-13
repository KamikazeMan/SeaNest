using Rhino.Geometry;

namespace SeaNest.Geometry
{
    /// <summary>
    /// Phase 20a — thin-plate metadata used by the joint-geometry
    /// algorithm. Computed by
    /// <see cref="JointGeometryHelpers.GetPlateInfo"/>; the
    /// mid-plane is the centerline between the two anti-parallel large
    /// faces, the normal is the plate's "thin" axis, and Thickness is
    /// the perpendicular separation between the faces.
    ///
    /// Phase 20a.8: moved out of <c>SeaNestRatHolesCommand</c> into the
    /// shared Geometry namespace so SeaNestSlotsCommand can reuse it
    /// without duplicating the type.
    /// </summary>
    public class PlateInfo
    {
        public Plane MidPlane;
        public Vector3d Normal;
        public double Thickness;
    }
}
