// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.lobstahbots.units.*
import com.revrobotics.AbsoluteEncoder
import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.RelativeEncoder
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.EncoderConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import frc.robot.Constants
import frc.robot.subsystems.drive.SwerveModuleIO.ModuleIOInputs
import frc.robot.util.tempControl.MonitoredSparkMax
import frc.robot.util.tempControl.TemperatureMonitor

/**
 * Creates a new SwerveModule for real cases.
 *
 * @param moduleID             The module number (0-3).
 * @param angleMotorID         The CAN ID of the motor controlling the angle.
 * @param driveMotorID         The CAN ID of the motor controlling drive speed.
 * @param angularOffsetDegrees The offset angle in degrees.
 */
class SwerveModuleIOSparkMax(
    /**
     * Returns the module ID.
     * 
     * @return The ID number of the module (0-3).
     */
    val moduleID: Int, name: String?, angleMotorID: Int, driveMotorID: Int,
    angularOffsetDegrees: Double, inverted: Boolean
) : SwerveModuleIO {
    private val angleMotor: MonitoredSparkMax = MonitoredSparkMax(
        angleMotorID, SparkLowLevel.MotorType.kBrushless,
        "$name angle motor"
    )
    private val driveMotor: MonitoredSparkMax = MonitoredSparkMax(
        driveMotorID, SparkLowLevel.MotorType.kBrushless,
        "$name drive motor"
    )
    private val drivingEncoder: RelativeEncoder
    private val angleAbsoluteEncoder: AbsoluteEncoder
    private val angularOffset: Rotation2d
    private val monitor: TemperatureMonitor

    init {
        val driveEncoderConfig = EncoderConfig()
            .positionConversionFactor(Constants.SwerveConstants.DRIVING_ENCODER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(Constants.SwerveConstants.DRIVING_ENCODER_VELOCITY_CONVERSION_FACTOR)
        val driveMotorConfig = SparkMaxConfig().apply(driveEncoderConfig)
            .smartCurrentLimit(Constants.DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT).idleMode(IdleMode.kBrake)
            .voltageCompensation(12.0).inverted(false)
        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        val angleMotorConfig = SparkMaxConfig().smartCurrentLimit(Constants.DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake).voltageCompensation(12.0).inverted(inverted)
        angleMotorConfig.absoluteEncoder
            .positionConversionFactor(Constants.SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(Constants.SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR)
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        drivingEncoder = driveMotor.getEncoder()
        angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder()

        monitor = TemperatureMonitor(listOf<TemperatureMonitor.Monitorable>(driveMotor, angleMotor))

        this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees)
        drivingEncoder.position = 0.0
        resetEncoders()
    }

    /** Stops the module motors.  */
    fun stopMotors() {
        angleMotor.stopMotor()
        driveMotor.stopMotor()
    }

    /**
     * Sets the braking mode of the driving motor.
     * 
     * @param mode the [IdleMode] to set motors to.
     */
    override fun setDriveIdleMode(mode: IdleMode) {
        driveMotor.configure(
            SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        )
    }

    /**
     * Sets the braking mode of the turning motor.
     * 
     * @param mode the [IdleMode] to set motors to.
     */
    override fun setTurnIdleMode(mode: IdleMode) {
        angleMotor.configure(
            SparkMaxConfig().idleMode(mode), ResetMode.kNoResetSafeParameters,
            PersistMode.kPersistParameters
        )
    }

    /** Zeroes the drive encoder.  */
    fun resetEncoders() {
        drivingEncoder.position = 0.0
    }

    /**
     * Sets voltage of driving motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    override fun setDriveVoltage(volts: Double) =
        driveMotor.setVoltage(volts)


    /**
     * Sets voltage of turn motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    override fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        inputs.drivePosition = if (driveMotor.lastError == REVLibError.kOk)
            drivingEncoder.position.rotations
        else
            inputs.drivePosition
        inputs.driveVelocity = if (driveMotor.lastError == REVLibError.kOk)
            drivingEncoder.velocity.rotationsPerSecond
        else
            inputs.driveVelocity
        inputs.driveAppliedVoltage = driveMotor.appliedOutput.value * driveMotor.busVoltage.volts
        inputs.driveStatorCurrent = driveMotor.outputCurrent.amps
        inputs.driveSupplyCurrent = driveMotor.outputCurrent.amps * driveMotor.appliedOutput // estimate
        inputs.driveTemperature = driveMotor.motorTemperature.celsius

        inputs.turnAbsolutePosition = if (angleMotor.lastError == REVLibError.kOk)
            Rotation2d.fromRadians(-angleAbsoluteEncoder.position - angularOffset.radians)
        else
            inputs.turnPosition
        inputs.turnPosition = inputs.turnAbsolutePosition
        inputs.turnVelocity = if (angleMotor.lastError == REVLibError.kOk)
            angleAbsoluteEncoder.velocity.radiansPerSecond
        else
            inputs.turnVelocity
        inputs.turnAppliedVoltage = angleMotor.appliedOutput.value * angleMotor.busVoltage.volts
        inputs.turnCurrent = angleMotor.outputCurrent.amps
        inputs.turnTemperature = angleMotor.motorTemperature.celsius
        inputs.angularOffset = angularOffset
    }

    override fun periodic() = monitor.monitor()
}
