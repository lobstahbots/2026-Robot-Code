// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.lobstahbots.junction.AutoLogKt
import edu.wpi.first.math.geometry.Rotation2d

/** Add your docs here.  */
interface GyroIO {
    @AutoLogKt
    open class GyroIOInputs {
        var connected: Boolean = false
        var rollPosition: Rotation2d = Rotation2d()
        var pitchPosition: Rotation2d = Rotation2d()
        var yawPosition: Rotation2d = Rotation2d()
        var rollVelocity: Double = 0.0
        var pitchVelocity: Double = 0.0
        var yawVelocity: Double = 0.0
        var isCalibrating: Boolean = true
    }

    /* Zeroes the gyro. */
    fun zeroGyro() {}

    val isCalibrating: Boolean
        get() = false

    fun updateInputs(inputs: GyroIOInputs) {}
}