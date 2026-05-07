using System;

namespace SeaNest.Nesting.Core.Geometry
{
    /// <summary>
    /// Immutable axis-aligned 2D bounding box.
    /// Stored as (MinX, MinY, MaxX, MaxY) for fast intersection and containment tests.
    /// Callers are responsible for ensuring Min &lt;= Max; no runtime validation.
    /// </summary>
    public readonly struct BoundingBox2D : IEquatable<BoundingBox2D>
    {
        public double MinX { get; }
        public double MinY { get; }
        public double MaxX { get; }
        public double MaxY { get; }

        public BoundingBox2D(double minX, double minY, double maxX, double maxY)
        {
            MinX = minX;
            MinY = minY;
            MaxX = maxX;
            MaxY = maxY;
        }

        public static BoundingBox2D FromCorners(Point2D min, Point2D max)
        {
            return new BoundingBox2D(min.X, min.Y, max.X, max.Y);
        }

        /// <summary>
        /// Build the smallest box containing both points, regardless of their relative position.
        /// </summary>
        public static BoundingBox2D FromPoints(Point2D a, Point2D b)
        {
            return new BoundingBox2D(
                Math.Min(a.X, b.X),
                Math.Min(a.Y, b.Y),
                Math.Max(a.X, b.X),
                Math.Max(a.Y, b.Y));
        }

        /// <summary>
        /// An empty/invalid box. Use to initialize an aggregate before Union-ing boxes into it.
        /// </summary>
        public static BoundingBox2D Empty => new BoundingBox2D(
            double.PositiveInfinity, double.PositiveInfinity,
            double.NegativeInfinity, double.NegativeInfinity);

        public double Width => MaxX - MinX;
        public double Height => MaxY - MinY;
        public double Area => Width * Height;
        public Point2D Min => new Point2D(MinX, MinY);
        public Point2D Max => new Point2D(MaxX, MaxY);
        public Point2D Center => new Point2D((MinX + MaxX) * 0.5, (MinY + MaxY) * 0.5);

        /// <summary>
        /// Diagonal length (useful for size-based sorting).
        /// </summary>
        public double Diagonal => Math.Sqrt(Width * Width + Height * Height);

        /// <summary>
        /// True if this is a valid non-empty box (width and height are both non-negative).
        /// </summary>
        public bool IsValid => MaxX >= MinX && MaxY >= MinY;

        /// <summary>
        /// Standard AABB intersection test — true if boxes overlap or touch.
        /// Used as a cheap early-reject before polygon-level checks.
        /// </summary>
        public bool Intersects(BoundingBox2D other)
        {
            return !(other.MinX > MaxX || other.MaxX < MinX ||
                     other.MinY > MaxY || other.MaxY < MinY);
        }

        /// <summary>
        /// Same as Intersects, but with a tolerance margin. Useful when we want
        /// to early-reject only boxes that are clearly apart.
        /// </summary>
        public bool IntersectsWithTolerance(BoundingBox2D other, double tolerance)
        {
            return !(other.MinX > MaxX + tolerance || other.MaxX < MinX - tolerance ||
                     other.MinY > MaxY + tolerance || other.MaxY < MinY - tolerance);
        }

        /// <summary>
        /// True if this box fully contains the other box (edges may touch).
        /// </summary>
        public bool Contains(BoundingBox2D other)
        {
            return other.MinX >= MinX && other.MaxX <= MaxX &&
                   other.MinY >= MinY && other.MaxY <= MaxY;
        }

        /// <summary>
        /// True if the point lies within the box (edges inclusive).
        /// </summary>
        public bool Contains(Point2D point)
        {
            return point.X >= MinX && point.X <= MaxX &&
                   point.Y >= MinY && point.Y <= MaxY;
        }

        /// <summary>
        /// Return a new box grown outward by 'amount' in every direction.
        /// Pass a negative amount to shrink (caller is responsible for ensuring result is still valid).
        /// </summary>
        public BoundingBox2D Inflated(double amount)
        {
            return new BoundingBox2D(
                MinX - amount, MinY - amount,
                MaxX + amount, MaxY + amount);
        }

        /// <summary>
        /// Return a new box that contains both this and the other box.
        /// Handles Empty correctly — Empty.Union(x) == x.
        /// </summary>
        public BoundingBox2D Union(BoundingBox2D other)
        {
            return new BoundingBox2D(
                Math.Min(MinX, other.MinX),
                Math.Min(MinY, other.MinY),
                Math.Max(MaxX, other.MaxX),
                Math.Max(MaxY, other.MaxY));
        }

        /// <summary>
        /// Return a new box extended to include the given point.
        /// </summary>
        public BoundingBox2D Union(Point2D p)
        {
            return new BoundingBox2D(
                Math.Min(MinX, p.X),
                Math.Min(MinY, p.Y),
                Math.Max(MaxX, p.X),
                Math.Max(MaxY, p.Y));
        }

        /// <summary>
        /// Return a new box translated by (dx, dy).
        /// </summary>
        public BoundingBox2D Translate(double dx, double dy)
        {
            return new BoundingBox2D(MinX + dx, MinY + dy, MaxX + dx, MaxY + dy);
        }

        public bool Equals(BoundingBox2D other)
        {
            return MinX == other.MinX && MinY == other.MinY &&
                   MaxX == other.MaxX && MaxY == other.MaxY;
        }

        public override bool Equals(object obj)
        {
            return obj is BoundingBox2D other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int h = MinX.GetHashCode();
                h = (h * 397) ^ MinY.GetHashCode();
                h = (h * 397) ^ MaxX.GetHashCode();
                h = (h * 397) ^ MaxY.GetHashCode();
                return h;
            }
        }

        public override string ToString()
        {
            return $"BBox[({MinX:F3},{MinY:F3})-({MaxX:F3},{MaxY:F3}), {Width:F3}x{Height:F3}]";
        }

        public static bool operator ==(BoundingBox2D a, BoundingBox2D b) => a.Equals(b);
        public static bool operator !=(BoundingBox2D a, BoundingBox2D b) => !a.Equals(b);
    }
}