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
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.measure.*
import frc.robot.Constants.IntakeConstants
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs

class IntakeIOTalonFX(armMotorID: Int, rollerMotorID: Int) : IntakeIO {
    private val armMotor: TalonFX = TalonFX(armMotorID)
    private val rollerMotor: TalonFX = TalonFX(rollerMotorID)

    private val armPosition: StatusSignal<Angle>
    private val armVelocity: StatusSignal<AngularVelocity>
    private val armTemp: StatusSignal<Temperature>
    private val armAppliedVoltage: StatusSignal<Voltage>
    private val armCurrent: StatusSignal<Current>
    private val rollerVelocity: StatusSignal<AngularVelocity>
    private val rollerTemp: StatusSignal<Temperature>
    private val rollerAppliedVoltage: StatusSignal<Voltage>
    private val rollerCurrent: StatusSignal<Current>

    private val controller = ProfiledPIDController(
        IntakeConstants.kP,
        IntakeConstants.kI,
        IntakeConstants.kD,
        TrapezoidProfile.Constraints(IntakeConstants.CRUISE_VELOCITY, IntakeConstants.MAX_ACCELERATION)
    )

    init {

        rollerMotor.configurator.apply(
            TalonFXConfiguration().withCurrentLimits(
                CurrentLimitsConfigs().withSupplyCurrentLimit(40.amps).withStatorCurrentLimit(
                    IntakeConstants.CURRENT_LIMIT
                )
            ).withMotorOutput(MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
        )

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

        rollerVelocity = rollerMotor.velocity
        rollerTemp = rollerMotor.deviceTemp
        rollerAppliedVoltage = rollerMotor.motorVoltage
        rollerCurrent = rollerMotor.statorCurrent

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
        rollerMotor.configurator.apply(MotorOutputConfigs().withNeutralMode(if (isBrake) NeutralModeValue.Brake else NeutralModeValue.Coast))
    }

    override fun resetEncoder(position: Rotation2d) {
        armMotor.setPosition(position.measure)
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        StatusSignal.refreshAll(
            armPosition,
            armVelocity,
            armAppliedVoltage,
            armCurrent,
            armTemp,
            armPosition,
            rollerVelocity,
            rollerAppliedVoltage,
            rollerTemp,
            rollerCurrent
        )

        armMotor.setVoltage(controller.calculate(armPosition.valueAsDouble))

        inputs.armVelocity = armVelocity.value
        inputs.armAppliedVoltage = armAppliedVoltage.value
        inputs.armCurrent = armCurrent.value
        inputs.armTemp = armTemp.value
        inputs.armPosition = Rotation2d(armPosition.value)

        inputs.rollerVelocity = rollerVelocity.value
        inputs.rollerAppliedVoltage = rollerAppliedVoltage.value
        inputs.rollerCurrent = rollerCurrent.value
        inputs.rollerTemp = rollerTemp.value
    }
}
