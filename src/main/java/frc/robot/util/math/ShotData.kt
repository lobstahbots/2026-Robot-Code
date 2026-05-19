package frc.robot.util.math

import com.lobstahbots.units.*
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.interpolation.InterpolatingTreeMap
import edu.wpi.first.math.interpolation.Interpolator
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.robot.FieldConstants
import frc.robot.util.trajectory.mirrorPose2d
import org.littletonrobotics.junction.Logger

/**
 * Represents the data for a shot; the flywheel velocity and hood position
 * necessary, and the time of flight for the ball during this shot.
 */
data class ShotData(val flywheelVelocity: AngularVelocity, val hoodPosition: Rotation2d, val tof: Time) {

    companion object {
        /**
         * Interpolate between two ShotData objects, with a slider from 0 to 1 choosing
         * where between the two the result is. The result is interpolated linearly
         * between the two start and end points.
         * 
         * @param a The lesser shot data
         * @param b The greater shot data
         * @param t Where in between them to go, `t` in [0, 1]
         * @return The interpolated ShotData
         * @see Interpolator
         */
        fun interpolate(a: ShotData, b: ShotData, t: Double): ShotData = ShotData(
            a.flywheelVelocity + (b.flywheelVelocity - a.flywheelVelocity) * t,
            a.hoodPosition.interpolate(b.hoodPosition, t),
            a.tof + (b.tof - a.tof) * t
        )

        /**
         * Do inverse interpolation between three [Distance] objects. That is,
         * given two endpoints of a range, and a point in between, return how far along
         * the point is between the two.
         * 
         * @param a The lower bound of the range
         * @param b The upper bound of the range
         * @param x The point in the middle of the range
         * @return how far along from `a` to `b` `x` is, a number in
         * [0, 1]
         * @see edu.wpi.first.math.interpolation.InverseInterpolator
         */
        fun inverseInterpolate(a: Distance, b: Distance, x: Distance): Double = x.div(b.minus(a)).baseUnitMagnitude()

        val shotMap: InterpolatingTreeMap<Distance, ShotData> = InterpolatingTreeMap(
            Companion::inverseInterpolate, Companion::interpolate
        )

        init {
            shotMap.put(
                1.35.meters, ShotData(3300.rpm, Rotation2d.fromDegrees(15.0), 1.second)
            )
            shotMap.put(
                1.6.meters, ShotData(3400.rpm, Rotation2d.fromDegrees(18.0), 1.second)
            )
            shotMap.put(
                1.95.meters, ShotData(3500.rpm, Rotation2d.fromDegrees(22.0), 1.second)
            )
            shotMap.put(
                2.5.meters, ShotData(3625.rpm, Rotation2d.fromDegrees(25.0), 1.second)
            )
            shotMap.put(
                2.92.meters, ShotData(3700.rpm, Rotation2d.fromDegrees(29.0), 1.second)
            )
            shotMap.put(
                3.35.meters, ShotData(3850.rpm, Rotation2d.fromDegrees(34.0), 1.second)
            )
            shotMap.put(
                3.95.meters, ShotData(3925.rpm, Rotation2d.fromDegrees(38.0), 1.second)
            )
            shotMap.put(
                4.58.meters, ShotData(4000.rpm, Rotation2d.fromDegrees(43.0), 1.second)
            )
            shotMap.put(
                4.91.meters, ShotData(4050.rpm, Rotation2d.fromDegrees(43.0), 1.second)
            )
            shotMap.put(
                5.40.meters, ShotData(4125.rpm, Rotation2d.fromDegrees(46.0), 1.second)
            )
        }

        fun getShotData(pose: Pose2d): ShotData {
            val dist =
                mirrorPose2d(pose).translation.getDistance(FieldConstants.Hub.innerCenterPoint.toTranslation2d()).meters
            Logger.recordOutput("AutoAim/Distance", dist)
            return shotMap[dist]
        }

        fun getShotData(distance: Double): ShotData = shotMap.get(distance.meters)

        val passMap: InterpolatingTreeMap<Distance, ShotData> =
            InterpolatingTreeMap(Companion::inverseInterpolate, Companion::interpolate)

        init {
            val ADD = 15.inches + FieldConstants.LeftBump.farLeftCorner.measureX
            passMap.put(
                0.00.meters.plus(ADD), ShotData(3100.rpm, Rotation2d.fromDegrees(46.0), 1.second)
            )
            passMap.put(
                1.00.meters.plus(ADD), ShotData(3300.rpm, Rotation2d.fromDegrees(46.0), 1.second)
            )
            passMap.put(
                2.04.meters.plus(ADD), ShotData(3500.rpm, Rotation2d.fromDegrees(46.0), 1.second)
            )
            passMap.put(
                2.73.meters.plus(ADD), ShotData(3650.rpm, Rotation2d.fromDegrees(48.0), 1.second)
            )
            passMap.put(
                3.63.meters.plus(ADD), ShotData(3850.rpm, Rotation2d.fromDegrees(48.0), 1.second)
            )
            passMap.put(
                4.67.meters.plus(ADD), ShotData(4000.rpm, Rotation2d.fromDegrees(48.0), 1.second)
            )
            passMap.put(
                6.meters.plus(ADD), ShotData(4500.rpm, Rotation2d.fromDegrees(50.0), 1.second)
            )
        }

        fun getPassData(pose: Pose2d): ShotData {
            val mirrored = mirrorPose2d(pose)
            val angle = mirrored.rotation + Rotation2d.kCW_Pi_2
            val dist = mirrored.measureX / angle.cos
            Logger.recordOutput("AutoAim/Pass", dist)
            return passMap[dist]
        }
    }
}
