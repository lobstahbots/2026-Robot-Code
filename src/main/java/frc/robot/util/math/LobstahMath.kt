// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.math

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Twist2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import kotlin.math.abs
import kotlin.math.sign

/**
 * Scales a number on a range of values to a corresponding value on a different
 * range
 *
 * @param x         The number to scale.
 * @param inputMin  The original range's lower bound
 * @param inputMax  The original range's upper bound
 * @param outputMin The new range's lower bound
 * @param outputMax The new range's upper bound
 */
fun scaleNumberToRange(
    x: Double, inputMin: Double, inputMax: Double, outputMin: Double, outputMax: Double
): Double {
    val inputRange = inputMax - inputMin
    val outputRange = outputMax - outputMin

    require(inputRange != 0.0) { "Input range cannot be 0" }

    return ((x - inputMin) / inputRange * outputRange) + outputMin
}

/**
 * Clamps and scales a number to a range of values to a corresponding value on a
 * different range
 *
 * @param x         The number to scale.
 * @param inputMin  The original range's lower bound
 * @param inputMax  The original range's upper bound
 * @param outputMin The new range's lower bound
 * @param outputMax The new range's upper bound
 */
fun scaleNumberToClampedRange(
    x: Double, inputMin: Double, inputMax: Double, outputMin: Double, outputMax: Double
): Double = scaleNumberToRange(MathUtil.clamp(x, inputMin, outputMax), inputMin, inputMax, outputMin, outputMax)

/**
 * Calculates turning output based on current and desired angle, for gyro values
 * clamped between 180 and -180 degrees.
 *
 * @param currentAngle The current gyro heading in degrees, 180 to -180.
 * @param desiredAngle The desired gyro heading in degrees, 180 to -180.
 */
fun calculateTurningOutput(currentAngle: Double, desiredAngle: Double): Double {
    var output = currentAngle - desiredAngle
    output %= 360.0
    if (abs(output) > 180) output -= sign(output) * 360
    return output
}

/**
 * Wraps value to fit within a range.
 *
 * @param lowThreshold  Lowest acceptable value.
 * @param highThreshold Highest acceptable value.
 */
fun wrapValue(value: Double, lowThreshold: Double, highThreshold: Double): Double {
    var value = value
    val range = highThreshold - lowThreshold

    if (value < lowThreshold) {
        while (value < lowThreshold) {
            value += range
        }
    }
    if (value > highThreshold) {
        value %= range
    }

    return value
}

/**
 * Unwraps an angle that has been previously wrapped from -pi to pi.
 *
 * @param ref   The reference angle
 * @param angle The angle to adjust
 * @return The adjusted angle
 */
fun unwrapAngle(ref: Double, angle: Double): Double {
    val diff = angle - ref
    return if (diff > Math.PI) {
        angle - 2.0 * Math.PI
    } else if (diff < -Math.PI) {
        angle + 2.0 * Math.PI
    } else {
        angle
    }
}

/**
 * Obtains a [Rotation2d] that points in the opposite direction from this
 * rotation.
 *
 * @return The rotation rotated by 180 degrees.
 */
fun flipRotation(rotation: Rotation2d): Rotation2d = rotation.rotateBy(Rotation2d.fromRadians(Math.PI))

/**
 * Converts a [ChassisSpeeds] object to a [Twist2d].
 *
 * @param chassisSpeeds The ChassisSpeeds to convert
 * @return A [Twist2d] representing the same `vxMetersPerSecond`, `vyMetersPerSecond`,
 * and `omegaRadiansPerSecond`.
 */
fun toTwist2d(chassisSpeeds: ChassisSpeeds): Twist2d = Twist2d(
    chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond
)

/**
 * Gets the distance between two [Pose2d]s.
 *
 * @param initialPose The first pose
 * @param endingPose  The second pose
 * @return The distance in meters
 */
fun getDistBetweenPoses(firstPose: Pose2d, secondPose: Pose2d): Double = firstPose.minus(secondPose).translation.norm

/**
 * Gets the 2D distance between a [Pose3d] and a [Pose2d].
 *
 * @param firstPose  The first pose
 * @param secondPose The second pose
 * @return The distance in meters
 */
fun getDistBetweenPoses(firstPose: Pose3d, secondPose: Pose2d): Double =
    getDistBetweenPoses(firstPose.toPose2d(), secondPose)

/**
 * Gets the 2D distance between a [Pose2d] and a [Pose3d].
 *
 * @param firstPose  The first pose
 * @param secondPose The second pose
 * @return The distance in meters
 */
fun getDistBetweenPoses(firstPose: Pose2d, secondPose: Pose3d): Double = getDistBetweenPoses(secondPose, firstPose)
