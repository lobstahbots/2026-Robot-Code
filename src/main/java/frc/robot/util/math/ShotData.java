package frc.robot.util.math;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.FieldConstants;
import frc.robot.util.trajectory.AlliancePoseMirror;

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

    static {
        Distance ADD = Inches.of(15).plus(Meters.of(FieldConstants.Hub.width).times(Math.sqrt(2) / 2));
        double mult = 3;
        shotMap.put(Meters.of(0.1).plus(ADD),
                new ShotData(RPM.of(1000).times(mult), Rotation2d.fromDegrees(17), Seconds.of(1)));
        shotMap.put(Meters.of(0.35).plus(ADD),
                new ShotData(RPM.of(1000).times(mult), Rotation2d.fromDegrees(20), Seconds.of(1)));
        shotMap.put(Meters.of(0.70).plus(ADD),
                new ShotData(RPM.of(1075).times(mult), Rotation2d.fromDegrees(24), Seconds.of(1)));
        shotMap.put(Meters.of(1.25).plus(ADD),
                new ShotData(RPM.of(1100).times(mult), Rotation2d.fromDegrees(27), Seconds.of(1)));
        shotMap.put(Meters.of(1.67).plus(ADD),
                new ShotData(RPM.of(1150).times(mult), Rotation2d.fromDegrees(29), Seconds.of(1)));
        shotMap.put(Meters.of(2.10).plus(ADD),
                new ShotData(RPM.of(1150).times(mult), Rotation2d.fromDegrees(32), Seconds.of(1)));
        shotMap.put(Meters.of(2.70).plus(ADD),
                new ShotData(RPM.of(1150).times(mult), Rotation2d.fromDegrees(36), Seconds.of(1)));
        shotMap.put(Meters.of(3.33).plus(ADD),
                new ShotData(RPM.of(1175).times(mult), Rotation2d.fromDegrees(39), Seconds.of(1)));
        shotMap.put(Meters.of(3.66).plus(ADD),
                new ShotData(RPM.of(1150).times(mult), Rotation2d.fromDegrees(42), Seconds.of(1)));
        shotMap.put(Meters.of(4.15).plus(ADD),
                new ShotData(RPM.of(1165).times(mult), Rotation2d.fromDegrees(44), Seconds.of(1)));
    }

    public static final ShotData getShotData(Pose2d pose) {
        return shotMap.get(Meters.of(AlliancePoseMirror.mirrorPose2d(pose).getTranslation()
                .getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d())));
    }

    public static final InterpolatingTreeMap<Distance, ShotData> passMap = new InterpolatingTreeMap<>(
            ShotData::inverseInterpolate, ShotData::interpolate);

    static {
        Distance ADD = Inches.of(15).plus(FieldConstants.LeftBump.farLeftCorner.getMeasureX());
        double mult = 3;
        passMap.put(Meters.of(0.00).plus(ADD),
                new ShotData(RPM.of(1000).times(mult), Rotation2d.fromDegrees(46), Seconds.of(1)));
        passMap.put(Meters.of(1.00).plus(ADD),
                new ShotData(RPM.of(1075).times(mult), Rotation2d.fromDegrees(46), Seconds.of(1)));
        passMap.put(Meters.of(2.04).plus(ADD),
                new ShotData(RPM.of(1150).times(mult), Rotation2d.fromDegrees(46), Seconds.of(1)));
        passMap.put(Meters.of(2.73).plus(ADD),
                new ShotData(RPM.of(1200).times(mult), Rotation2d.fromDegrees(48), Seconds.of(1)));
        passMap.put(Meters.of(3.63).plus(ADD),
                new ShotData(RPM.of(1250).times(mult), Rotation2d.fromDegrees(48), Seconds.of(1)));
        passMap.put(Meters.of(4.67).plus(ADD),
                new ShotData(RPM.of(1300).times(mult), Rotation2d.fromDegrees(48), Seconds.of(1)));
    }

    public static final ShotData getPassData(Pose2d pose) {
        Pose2d mirrored = AlliancePoseMirror.mirrorPose2d(pose);
        Rotation2d angle = mirrored.getRotation().plus(Rotation2d.kCW_Pi_2);
        return passMap.get(mirrored.getMeasureX().times(1 / angle.getCos()));
    }

}
