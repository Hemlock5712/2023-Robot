package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Translation2dPlus extends Translation2d {
  public static double dot(final Translation2d a, final Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  public static Rotation2d getAngle(final Translation2d a, final Translation2d b) {
    double cosAngle = dot(a, b) / (a.getNorm() * b.getNorm());

    if (Double.isNaN(cosAngle)) {
      return new Rotation2d();
    }

    return Rotation2d.fromRadians(Math.acos(Math.min(1.0, Math.max(cosAngle, -1.0))));
  }

  public static double cross(final Translation2d a, final Translation2d b) {
    return a.getX() * b.getY() - a.getY() * b.getY();
  }

  public Translation2dPlus() {
    super(0.0, 0.0);
  }

  public Translation2dPlus(final double x, final double y) {
    super(x, y);
  }

  public Translation2dPlus(final double distance, final Rotation2d direction) {
    super(distance, direction);
  }

  public Translation2dPlus(final Translation2d trans) {
    super(trans.getX(), trans.getY());
  }

  public Translation2dPlus(final Translation2d start, final Translation2d end) {
    super(start.getX() - end.getX(), start.getY() - end.getY());
  }

  /**
   * Normalizing a vector scales it so that its norm is 1 while maintaining its
   * direction.
   * If input is a zero vector, return a zero vector.
   *
   * @return r / norm(r) or (0,0)
   */
  public Translation2d normalized() {
    if (Math.abs(getX()) < 1e-9 && Math.abs(getY()) < 1e-5) {
      return this;
    } else {
      return times(1.0 / getNorm());
    }
  }

  /**
   * The scalar projection of a vector u onto a vector v is the length of
   * the "shadow" cast by u onto v under a "light" that is placed on a line
   * normal to v and containing the endpoint of u, given that u and v share
   * a starting point.
   * tl;dr:
   * _*
   * u /|
   * / |
   * / | v
   * *---+---------------->*
   * \___/
   * |
   * scal_v(u)
   * u.scal(v)
   *
   * @return (u . v) / norm(v)
   */
  public double scal(final Translation2d v) {
    return dot(this, v) / v.getNorm();
  }

  /**
   * The projection of a vector u onto a vector v is the vector in the direction
   * of v with the magnitude u.scal(v).
   *
   * @return u.scal(v) * v / norm(v)
   */
  public Translation2d proj(final Translation2dPlus v) {
    return v.normalized().times(scal(v));
  }

  public Translation2d proj(final Translation2d v) {
    return proj(new Translation2dPlus(v));
  }

  /**
   * https://stackoverflow.com/a/1167047/6627273
   * A point D is considered "within" an angle ABC when
   * cos(DBM) > cos(ABM)
   * where M is the midpoint of AC, so ABM is half the angle ABC.
   * The cosine of an angle can be computed as the dot product of two normalized
   * vectors in the directions of its sides.
   * Note that this definition of "within" does not include points that lie on
   * the sides of the given angle.
   * If `vertical` is true, then check not within the given angle, but within the
   * image of that angle rotated by pi about its vertex.
   *
   * @param A        A point on one side of the angle.
   * @param B        The vertex of the angle.
   * @param C        A point on the other side of the angle.
   * @param vertical Whether to check in the angle vertical to the one given
   * @return Whether this translation is within the given angle.
   * @author Joseph Reed
   */
  public boolean isWithinAngle(
      final Translation2d A,
      final Translation2d B,
      final Translation2d C,
      final boolean vertical) {
    final var M = A.interpolate(C, 0.5); // midpoint
    var m = new Translation2dPlus(B, M).normalized(); // mid-vector
    var a = new Translation2dPlus(B, A).normalized(); // side vector
    final var d = new Translation2dPlus(B, this).normalized(); // vector to here

    if (vertical) {
      m = m.unaryMinus();
      a = a.unaryMinus();
    }

    return Translation2dPlus.dot(d, m) > Translation2dPlus.dot(a, m);
  }

  public boolean isWithinAngle(final Translation2d A, final Translation2d B, final Translation2d C) {
    return isWithinAngle(A, B, C, false);
  }

  /** Assumes an angle centered at the origin. */
  public boolean isWithinAngle(final Translation2d A, final Translation2d C, final boolean vertical) {
    return isWithinAngle(A, new Translation2d(), C, vertical);
  }

  public boolean isWithinAngle(final Translation2d A, final Translation2d C) {
    return isWithinAngle(A, C, false);
  }

  /**
   * The distance between a point and a line can be computed as a scalar
   * projection.
   *
   * @param a One point on the line.
   * @param b Another point on the line.
   */
  public double distanceToLine(final Translation2d a, final Translation2d b) {
    final var point = new Translation2dPlus(a, this);
    final var line = new Translation2dPlus(a, b);
    final var perpLine = line.rotateBy(new Rotation2d(90));
    return Math.abs(point.scal(perpLine));
  }
}
