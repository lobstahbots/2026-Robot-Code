// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.drive

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.AbsoluteEncoder
import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import com.lobstahbots.units.*
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import frc.robot.Constants.DriveConstants
import frc.robot.Constants.RobotConstants
import frc.robot.Constants.SwerveConstants
import frc.robot.subsystems.drive.SwerveModuleIO.ModuleIOInputs
import frc.robot.util.tempControl.MonitoredSparkMax
import frc.robot.util.tempControl.TemperatureMonitor
import java.util.*

/**
 * Creates a new SwerveModule for real cases.
 *
 * @param moduleID             The module number (0-3).
 * @param angleMotorID         The CAN ID of the motor controlling the angle.
 * @param driveMotorID         The CAN ID of the motor controlling drive speed.
 * @param angularOffsetDegrees The offset angle in degrees.
 */
class SwerveModuleIOTalonFX(
    /**
     * Returns the module ID.
     * 
     * @return The ID number of the module (0-3).
     */
    val moduleID: Int, name: String?, angleMotorID: Int, driveMotorID: Int,
    angularOffsetDegrees: Double, inverted: Boolean
) : SwerveModuleIO {
    private val angleMotor: MonitoredSparkMax =
        MonitoredSparkMax(angleMotorID, SparkLowLevel.MotorType.kBrushless, "$name angle motor")
    private val driveMotor: TalonFX = TalonFX(driveMotorID)
    private val angleAbsoluteEncoder: AbsoluteEncoder
    private val angularOffset: Rotation2d
    private val monitor: TemperatureMonitor
    private val driveVelocity: StatusSignal<AngularVelocity>
    private val drivePosition: StatusSignal<Angle>
    private val driveVoltage: StatusSignal<Voltage>
    private val driveCurrent: StatusSignal<Current>
    private val driveSupply: StatusSignal<Current>
    private val driveTemperature: StatusSignal<Temperature>

    init {
        driveVelocity = driveMotor.velocity
        drivePosition = driveMotor.position
        driveVoltage = driveMotor.motorVoltage
        driveCurrent = driveMotor.statorCurrent
        driveSupply = driveMotor.supplyCurrent
        driveTemperature = driveMotor.deviceTemp

        val driveMotorConfig = TalonFXConfiguration()
            .withCurrentLimits(
                CurrentLimitsConfigs().withSupplyCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT)
                    .withStatorCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT * 3.0)
            )
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(RobotConstants.DRIVE_GEAR_RATIO))
            .withMotorOutput(MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        driveMotor.configurator.apply(driveMotorConfig)

        val angleMotorConfig = SparkMaxConfig().smartCurrentLimit(DriveConstants.ANGLE_MOTOR_CURRENT_LIMIT)
            .idleMode(IdleMode.kBrake).voltageCompensation(12.0).inverted(inverted)
        angleMotorConfig.absoluteEncoder
            .positionConversionFactor(SwerveConstants.TURNING_ENCODER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(SwerveConstants.TURNING_ENCODER_VELOCITY_CONVERSION_FACTOR)
        angleMotor.configure(angleMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        angleAbsoluteEncoder = angleMotor.getAbsoluteEncoder()

        monitor = TemperatureMonitor(listOf<TemperatureMonitor.Monitorable>(angleMotor))

        this.angularOffset = Rotation2d.fromDegrees(angularOffsetDegrees)
        driveMotor.setPosition(0.0)
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
        driveMotor.configurator.apply(
            MotorOutputConfigs()
                .withNeutralMode(if (mode == IdleMode.kBrake) NeutralModeValue.Brake else NeutralModeValue.Coast)
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
        driveMotor.setPosition(0.0)
    }

    /**
     * Sets voltage of driving motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    override fun setDriveVoltage(volts: Double) {
        driveMotor.setVoltage(volts)
    }

    /**
     * Sets voltage of turn motor.
     * 
     * @param volts The voltage the motor should be set to.
     */
    override fun setTurnVoltage(volts: Double) {
        angleMotor.setVoltage(volts)
    }

    override fun updateInputs(inputs: ModuleIOInputs) {
        BaseStatusSignal.refreshAll(
            drivePosition,
            driveVelocity,
            driveVoltage,
            driveCurrent,
            driveSupply,
            driveTemperature
        )

        inputs.drivePosition = drivePosition.value
        inputs.driveVelocity = driveVelocity.value
        inputs.driveAppliedVoltage = driveVoltage.value
        inputs.driveStatorCurrent = driveCurrent.value
        inputs.driveSupplyCurrent = driveSupply.value
        inputs.driveTemperature = driveTemperature.value

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
        inputs.angularOffset = angularOffset
        inputs.turnTemperature = angleMotor.motorTemperature.celsius
    }

    override fun periodic() = monitor.monitor()
}
