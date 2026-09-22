// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake

import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import com.lobstahbots.units.*
import com.revrobotics.PersistMode
import com.revrobotics.RelativeEncoder
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import frc.robot.Constants.IntakeConstants
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs

class IntakeIOTalonFX(armMotorID: Int, rollerMotorID: Int) : IntakeIO {
    private val armMotor: TalonFX = TalonFX(armMotorID)
    private val rollerMotor: SparkMax = SparkMax(rollerMotorID, SparkLowLevel.MotorType.kBrushless)

    private val rollerEncoder: RelativeEncoder

    private val armPosition: StatusSignal<Angle>
    private val armVelocity: StatusSignal<AngularVelocity>
    private val armTemp: StatusSignal<Temperature>
    private val armAppliedVoltage: StatusSignal<Voltage>
    private val armCurrent: StatusSignal<Current>

    private val controller = ProfiledPIDController(
        IntakeConstants.kP,
        IntakeConstants.kI,
        IntakeConstants.kD,
        TrapezoidProfile.Constraints(IntakeConstants.CRUISE_VELOCITY, IntakeConstants.MAX_ACCELERATION)
    )

    init {
        val config = SparkMaxConfig()

        config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake)
            .inverted(true).encoder.velocityConversionFactor(1 / 60.0)

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        val armConfig = TalonFXConfiguration()

        armConfig.withCurrentLimits(
            CurrentLimitsConfigs().withSupplyCurrentLimit(40.amps).withStatorCurrentLimit(80.amps)
        ).Feedback.withSensorToMechanismRatio(IntakeConstants.GEAR_RATIO)

        armMotor.configurator.apply(armConfig)

        armPosition = armMotor.position
        armVelocity = armMotor.velocity
        armTemp = armMotor.deviceTemp
        armAppliedVoltage = armMotor.motorVoltage
        armCurrent = armMotor.statorCurrent

        rollerEncoder = rollerMotor.getEncoder()

        resetEncoder(IntakeConstants.STOWED)
    }

    override fun stopArmMotor() = armMotor.stopMotor()

    override fun stopRollerMotor() = rollerMotor.stopMotor()

    override fun setArmVoltage(volts: Double) = armMotor.setVoltage(volts)

    override fun setRollerVoltage(volts: Double) = rollerMotor.setVoltage(volts)

    override fun setArmPosition(position: Rotation2d) {
        controller.goal = TrapezoidProfile.State(position.rotations, 0.0)
    }

    override fun setRollerSpeed(speed: Double) = rollerMotor.set(speed)

    override fun setArmIdleMode(isBrake: Boolean) {
        armMotor.configurator.apply(MotorOutputConfigs().withNeutralMode(if (isBrake) NeutralModeValue.Brake else NeutralModeValue.Coast))
    }

    override fun setRollerIdleMode(isBrake: Boolean) {
        val config = SparkMaxConfig()
        config.idleMode(if (isBrake) IdleMode.kBrake else IdleMode.kCoast)
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun resetEncoder(position: Rotation2d) {
        armMotor.setPosition(position.measure)
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        StatusSignal.refreshAll(armPosition, armVelocity, armAppliedVoltage, armCurrent, armTemp, armPosition)
        armMotor.setVoltage(controller.calculate(armPosition.valueAsDouble))
        inputs.armVelocity = armVelocity.value
        inputs.armAppliedVoltage = armAppliedVoltage.value
        inputs.armCurrent = armCurrent.value
        inputs.armTemp = armTemp.value
        inputs.armPosition = Rotation2d(armPosition.value)

        inputs.rollerVelocity = rollerEncoder.velocity.rotationsPerSecond
        inputs.rollerAppliedVoltage = rollerMotor.appliedOutput.value * rollerMotor.busVoltage.volts
        inputs.rollerCurrent = rollerMotor.outputCurrent.amps
        inputs.rollerTemp = rollerMotor.motorTemperature.celsius
    }
}
