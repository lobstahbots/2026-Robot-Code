package frc.robot.util.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * Represents the data for a shot; the flywheel velocity and hood position
 * necessary, and the time of flight for the ball during this shot.
 */
public record ShotData(AngularVelocity flywheelVelocity, Rotation2d hoodPosition, Time tof) {
    /**
     * Interpolate between two ShotData objects, with a slider from 0 to 1 choosing
     * where between the two the result is. The result is interpolated linearly
     * between the two start and end points.
     * 
     * @param a The lesser shot data
     * @param b The greater shot data
     * @param t Where in between them to go, {@code t} in [0, 1]
     * @return The interpolated ShotData
     * @see edu.wpi.first.math.interpolation.Interpolator
     */
    public static ShotData interpolate(ShotData a, ShotData b, double t) {
        return new ShotData(a.flywheelVelocity.plus(b.flywheelVelocity.minus(a.flywheelVelocity).times(t)),
                a.hoodPosition.interpolate(b.hoodPosition, t), a.tof.plus(b.tof.minus(a.tof).times(t)));
    }

    /**
     * Do inverse interpolation between three {@link Distance} objects. That is,
     * given two endpoints of a range, and a point in between, return how far along
     * the point is between the two.
     * 
     * @param a The lower bound of the range
     * @param b The upper bound of the range
     * @param x The point in the middle of the range
     * @return how far along from {@code a} to {@code b} {@code x} is, a number in
     *         [0, 1]
     * @see edu.wpi.first.math.interpolation.InverseInterpolator
     */
    public static double inverseInterpolate(Distance a, Distance b, Distance x) {
        return x.div(b.minus(a)).baseUnitMagnitude();
    }

    public static final InterpolatingTreeMap<Distance, ShotData> shotMap = new InterpolatingTreeMap<>(
            ShotData::inverseInterpolate, ShotData::interpolate);
}
