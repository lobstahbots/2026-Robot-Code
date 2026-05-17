// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.lobstahbots.junction.AutoLogKt
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModuleState
import com.lobstahbots.units.*

interface SwerveModuleIO {
    @AutoLogKt
    open class ModuleIOInputs {
        var drivePosition = 0.rotations
        var driveVelocity = 0.radiansPerSecond
        var driveAppliedVoltage = 0.volts
        var driveStatorCurrent = 0.amps
        var driveSupplyCurrent = 0.amps
        var driveTemperature = 25.celsius

        var turnAbsolutePosition = Rotation2d()
        var turnPosition = Rotation2d()
        var turnVelocity = 0.radiansPerSecond
        var turnAppliedVoltage = 0.volts
        var turnCurrent = 0.amps
        var turnTemperature = 25.celsius
        var angularOffset = Rotation2d()
    }

    fun updateInputs(inputs: ModuleIOInputs) {}

    /** Run the drive motor at the specified voltage.  */
    fun setDriveVoltage(volts: Double) {}

    /** Run the turn motor at the specified voltage.  */
    fun setTurnVoltage(volts: Double) {}

    /** Set the angle to the angle specified in the module state.  */
    fun setAngle(optimizedDesiredState: SwerveModuleState) {}

    /** Set the drive speed to the angle specified in the module state.  */
    fun setDriveSpeed(optimizedDesiredState: SwerveModuleState, isOpenLoop: Boolean) {}

    /** Enable or disable brake mode on the drive motor.  */
    fun setDriveIdleMode(mode: IdleMode) {}

    /** Enable or disable brake mode on the turn motor.  */
    fun setTurnIdleMode(mode: IdleMode) {}

    fun periodic() {}
}
