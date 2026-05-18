// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake

import com.lobstahbots.units.*
import com.revrobotics.PersistMode
import com.revrobotics.RelativeEncoder
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.FeedForwardConfig
import com.revrobotics.spark.config.MAXMotionConfig
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.robot.Constants.IntakeConstants
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs

class IntakeIOSparkMax(armMotorID: Int, rollerMotorID: Int) : IntakeIO {
    private val armMotor: SparkMax = SparkMax(armMotorID, SparkLowLevel.MotorType.kBrushless)
    private val rollerMotor: SparkMax = SparkMax(rollerMotorID, SparkLowLevel.MotorType.kBrushless)

    private val armEncoder: RelativeEncoder
    private val rollerEncoder: RelativeEncoder

    private val controller = ProfiledPIDController(
        IntakeConstants.kP, IntakeConstants.kI,
        IntakeConstants.kD,
        TrapezoidProfile.Constraints(IntakeConstants.CRUISE_VELOCITY, IntakeConstants.MAX_ACCELERATION)
    )

    init {
        val config = SparkMaxConfig()

        config.smartCurrentLimit(IntakeConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true).encoder
            .velocityConversionFactor(1 / 60.0)

        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        config.inverted(false).encoder.positionConversionFactor(1.0 / IntakeConstants.GEAR_RATIO)
            .velocityConversionFactor(1 / 60.0 / IntakeConstants.GEAR_RATIO)
        config.closedLoop.pid(IntakeConstants.kP, IntakeConstants.kI, IntakeConstants.kD)
            .apply(
                FeedForwardConfig().svacr(
                    IntakeConstants.kS, IntakeConstants.kV, IntakeConstants.kA,
                    IntakeConstants.kG, 1.0
                )
            )
            .apply(
                MAXMotionConfig().cruiseVelocity(IntakeConstants.CRUISE_VELOCITY)
                    .maxAcceleration(IntakeConstants.MAX_ACCELERATION)
            )
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        armEncoder = armMotor.getEncoder()
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
        val config = SparkMaxConfig()
        config.idleMode(if (isBrake) IdleMode.kBrake else IdleMode.kCoast)
        armMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun setRollerIdleMode(isBrake: Boolean) {
        val config = SparkMaxConfig()
        config.idleMode(if (isBrake) IdleMode.kBrake else IdleMode.kCoast)
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun resetEncoder(position: Rotation2d) {
        armEncoder.position = position.rotations
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        armMotor.setVoltage(controller.calculate(armEncoder.position))
        inputs.armVelocity = armEncoder.velocity.rotationsPerSecond
        inputs.armAppliedVoltage = armMotor.appliedOutput.value * armMotor.busVoltage.volts
        inputs.armCurrent = armMotor.outputCurrent.amps
        inputs.armTemp = armMotor.motorTemperature.celsius
        inputs.armPosition = Rotation2d.fromRotations(armEncoder.position)

        inputs.rollerVelocity = rollerEncoder.velocity.rotationsPerSecond
        inputs.rollerAppliedVoltage = rollerMotor.appliedOutput.value * rollerMotor.busVoltage.volts
        inputs.rollerCurrent = rollerMotor.outputCurrent.amps
        inputs.rollerTemp = rollerMotor.motorTemperature.celsius
    }
}
