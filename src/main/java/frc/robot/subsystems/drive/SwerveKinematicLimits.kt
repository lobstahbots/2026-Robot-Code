// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

/** Add your docs here.  */
@JvmRecord
data class SwerveKinematicLimits(
    val maxDriveVelocity: Double,
    val maxDriveAcceleration: Double,
    val maxSteeringVelocity: Double
)
