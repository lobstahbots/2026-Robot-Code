// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.util.trajectory

import edu.wpi.first.math.geometry.*
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.robot.FieldConstants

/**
 * Returns mirrored [Translation2d] depending on alliance color.
 *
 * @param translation The [Translation2d] to mirror
 * @return A corrected [Translation2d] (mirrored or unmirrored) depending on
 * alliance color.
 */
fun mirrorTranslation2d(translation: Translation2d): Translation2d {
    return if (isRedAlliance) {
        FieldConstants.FIELD_CENTER + FieldConstants.FIELD_CENTER - translation
    } else {
        translation
    }
}

/**
 * Returns mirrored [Translation3d] depending on alliance color.
 *
 * @param translation The [Translation3d] to mirror
 * @return A corrected [Translation3d] (mirrored or unmirrored) depending on
 * alliance color.
 */
fun mirrorTranslation3d(translation: Translation3d): Translation3d {
    return if (isRedAlliance) {
        Translation3d(mirrorTranslation2d(translation.toTranslation2d())) + Translation3d(0.0, 0.0, translation.z)
    } else {
        translation
    }
}

/**
 * Returns mirrored [Rotation2d] depending on alliance color.
 *
 * @param rotation The [Rotation2d] to mirror
 * @return A corrected [Rotation2d] (mirrored or unmirrored) depending on alliance
 * color.
 */
fun mirrorRotation2d(rotation: Rotation2d): Rotation2d {
    return if (isRedAlliance) {
        rotation.rotateBy(Rotation2d.k180deg)
    } else {
        rotation
    }
}

/**
 * Returns mirrored [Rotation3d] depending on alliance color.
 *
 * @param rotation The [Rotation3d] to mirror
 * @return A corrected [Rotation3d] (mirrored or unmirrored) depending on alliance
 * color.
 */
fun mirrorRotation3d(rotation: Rotation3d): Rotation3d {
    return if (isRedAlliance) {
        Rotation3d(rotation.x, rotation.y, rotation.z + Math.PI)
    } else {
        rotation
    }
}

/**
 * Returns mirrored [Pose2d] depending on alliance color.
 *
 * @param pose The [Pose2d] to mirror
 * @return A corrected [Pose2d] (mirrored or unmirrored) depending on alliance
 * color.
 */
fun mirrorPose2d(pose: Pose2d): Pose2d {
    return if (isRedAlliance) {
        Pose2d(mirrorTranslation2d(pose.translation), mirrorRotation2d(pose.rotation))
    } else {
        pose
    }
}

/**
 * Returns mirrored [Pose3d] depending on alliance color.
 *
 * @param pose The [Pose3d] to mirror
 * @return A corrected [Pose3d] (mirrored or unmirrored) depending on alliance
 * color.
 */
fun mirrorPose3d(pose: Pose3d): Pose3d {
    return if (isRedAlliance) {
        Pose3d(mirrorTranslation3d(pose.translation), mirrorRotation3d(pose.rotation))
    } else {
        pose
    }
}

fun flipRotation(rotation: Rotation2d): Rotation2d {
    if (isRedAlliance) {
        return flipRotation(rotation)
    }
    return rotation
}

val isRedAlliance: Boolean
    /**
     * @return Whether the robot is on the red alliance side and positions should be
     * mirrored.
     */
    get() = DriverStation.getAlliance().isPresent && DriverStation.getAlliance().get() == Alliance.Red
