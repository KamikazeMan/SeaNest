using System;

namespace SeaNest.Nesting.Core.Geometry
{
    /// <summary>
    /// Immutable 2D point with double precision.
    /// Used as the fundamental primitive for all nesting geometry.
    /// </summary>
    public readonly struct Point2D : IEquatable<Point2D>
    {
        public double X { get; }
        public double Y { get; }

        public Point2D(double x, double y)
        {
            X = x;
            Y = y;
        }

        public static Point2D Origin => new Point2D(0, 0);

        public double DistanceTo(Point2D other)
        {
            double dx = other.X - X;
            double dy = other.Y - Y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        public double DistanceSquaredTo(Point2D other)
        {
            double dx = other.X - X;
            double dy = other.Y - Y;
            return dx * dx + dy * dy;
        }

        public Point2D Translate(double dx, double dy)
        {
            return new Point2D(X + dx, Y + dy);
        }

        public Point2D RotateAround(Point2D center, double angleRadians)
        {
            double cos = Math.Cos(angleRadians);
            double sin = Math.Sin(angleRadians);
            double dx = X - center.X;
            double dy = Y - center.Y;
            return new Point2D(
                center.X + dx * cos - dy * sin,
                center.Y + dx * sin + dy * cos);
        }

        public bool Equals(Point2D other)
        {
            return X == other.X && Y == other.Y;
        }

        public bool EqualsWithTolerance(Point2D other, double tolerance)
        {
            return Math.Abs(X - other.X) <= tolerance &&
                   Math.Abs(Y - other.Y) <= tolerance;
        }

        public override bool Equals(object obj)
        {
            return obj is Point2D other && Equals(other);
        }

        public override int GetHashCode()
        {
            // Combine X and Y hashes - standard pattern for value types
            unchecked
            {
                return (X.GetHashCode() * 397) ^ Y.GetHashCode();
            }
        }

        public override string ToString()
        {
            return $"({X:F4}, {Y:F4})";
        }

        public static bool operator ==(Point2D a, Point2D b) => a.Equals(b);
        public static bool operator !=(Point2D a, Point2D b) => !a.Equals(b);
    }
}