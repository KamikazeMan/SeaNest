using System;

namespace SeaNest.Nesting.Core.Geometry
{
    /// <summary>
    /// Immutable 2D affine transform consisting of a rotation followed by a translation.
    /// Applies as: p -> R*p + t, where R is a 2x2 rotation matrix and t a translation vector.
    /// No scale, no skew. Angles are CCW positive (standard math convention).
    /// </summary>
    public readonly struct Transform2D : IEquatable<Transform2D>
    {
        /// <summary>Cosine of the rotation angle.</summary>
        public double Cos { get; }

        /// <summary>Sine of the rotation angle.</summary>
        public double Sin { get; }

        /// <summary>X component of the translation.</summary>
        public double Tx { get; }

        /// <summary>Y component of the translation.</summary>
        public double Ty { get; }

        /// <summary>
        /// Direct constructor from rotation matrix components and translation.
        /// For a valid rotation, Cos*Cos + Sin*Sin should equal 1. This is not validated;
        /// callers should use the static factory methods unless they know what they're doing.
        /// </summary>
        public Transform2D(double cos, double sin, double tx, double ty)
        {
            Cos = cos;
            Sin = sin;
            Tx = tx;
            Ty = ty;
        }

        /// <summary>The identity transform: no rotation, no translation.</summary>
        public static Transform2D Identity => new Transform2D(1.0, 0.0, 0.0, 0.0);

        /// <summary>Pure translation by (dx, dy).</summary>
        public static Transform2D Translation(double dx, double dy)
            => new Transform2D(1.0, 0.0, dx, dy);

        /// <summary>Pure translation by the given offset, interpreted as a vector.</summary>
        public static Transform2D Translation(Point2D offset)
            => new Transform2D(1.0, 0.0, offset.X, offset.Y);

        /// <summary>Rotation about the origin by the given angle in radians (CCW positive).</summary>
        public static Transform2D Rotation(double radians)
            => new Transform2D(Math.Cos(radians), Math.Sin(radians), 0.0, 0.0);

        /// <summary>Rotation about the origin by the given angle in degrees (CCW positive).</summary>
        public static Transform2D RotationDegrees(double degrees)
            => Rotation(degrees * Math.PI / 180.0);

        /// <summary>
        /// Rotation about an arbitrary center point by the given angle in radians (CCW positive).
        /// Equivalent to Translation(center) applied after Rotation(radians) applied after Translation(-center).
        /// </summary>
        public static Transform2D RotationAround(Point2D center, double radians)
        {
            double c = Math.Cos(radians);
            double s = Math.Sin(radians);
            // p -> R*(p - center) + center  =>  R*p + (center - R*center)
            double tx = center.X - (c * center.X - s * center.Y);
            double ty = center.Y - (s * center.X + c * center.Y);
            return new Transform2D(c, s, tx, ty);
        }

        /// <summary>
        /// Rotation about an arbitrary center point by the given angle in degrees (CCW positive).
        /// </summary>
        public static Transform2D RotationAroundDegrees(Point2D center, double degrees)
            => RotationAround(center, degrees * Math.PI / 180.0);

        /// <summary>
        /// Apply this transform to a point. Returns R*p + t.
        /// </summary>
        public Point2D Apply(Point2D p)
            => new Point2D(
                Cos * p.X - Sin * p.Y + Tx,
                Sin * p.X + Cos * p.Y + Ty);

        /// <summary>
        /// Compose two transforms. The returned transform applies <c>this</c> first, then <paramref name="next"/>.
        /// Reads left-to-right: <c>a.Then(b).Then(c)</c> means apply a, then b, then c.
        /// </summary>
        public Transform2D Then(Transform2D next)
        {
            // (next o this)(p) = next.R * (this.R * p + this.t) + next.t
            //                  = (next.R * this.R) * p + (next.R * this.t + next.t)
            double newCos = next.Cos * this.Cos - next.Sin * this.Sin;
            double newSin = next.Sin * this.Cos + next.Cos * this.Sin;
            double newTx = next.Cos * this.Tx - next.Sin * this.Ty + next.Tx;
            double newTy = next.Sin * this.Tx + next.Cos * this.Ty + next.Ty;
            return new Transform2D(newCos, newSin, newTx, newTy);
        }

        /// <summary>
        /// The rotation angle recovered from the rotation matrix, in radians, in the range (-pi, pi].
        /// </summary>
        public double RotationRadians => Math.Atan2(Sin, Cos);

        /// <summary>
        /// The rotation angle recovered from the rotation matrix, in degrees, in the range (-180, 180].
        /// </summary>
        public double RotationAngleDegrees => RotationRadians * 180.0 / Math.PI;

        public bool Equals(Transform2D other)
            => Cos == other.Cos && Sin == other.Sin && Tx == other.Tx && Ty == other.Ty;

        public override bool Equals(object obj)
            => obj is Transform2D t && Equals(t);

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + Cos.GetHashCode();
                hash = hash * 31 + Sin.GetHashCode();
                hash = hash * 31 + Tx.GetHashCode();
                hash = hash * 31 + Ty.GetHashCode();
                return hash;
            }
        }

        public static bool operator ==(Transform2D a, Transform2D b) => a.Equals(b);
        public static bool operator !=(Transform2D a, Transform2D b) => !a.Equals(b);

        public override string ToString()
            => $"Transform2D(rot={RotationAngleDegrees:F3}°, t=({Tx:F3}, {Ty:F3}))";
    }
}