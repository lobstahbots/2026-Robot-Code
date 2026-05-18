// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.intake

import com.lobstahbots.units.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import frc.robot.Constants.IntakeConstants
import frc.robot.Constants.SimConstants
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs

/** Add your docs here.  */
class IntakeIOSimBasic : IntakeIO {
    private var state = TrapezoidProfile.State(IntakeConstants.STOWED.rotations, 0.0)
    private var rollerSpeed = 0.0
    private val profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(IntakeConstants.CRUISE_VELOCITY, IntakeConstants.MAX_ACCELERATION)
    )
    private var goal: TrapezoidProfile.State = state

    override fun stopArmMotor() {
        goal = TrapezoidProfile.State(state.position, 0.0)
    }

    override fun setRollerSpeed(speed: Double) {
        rollerSpeed = 20 * speed
    }

    override fun stopRollerMotor() {
        rollerSpeed = 0.0
    }

    override fun setRollerVoltage(volts: Double) {
        setRollerSpeed(volts / 12.0)
    }

    override fun setArmPosition(position: Rotation2d) {
        goal = TrapezoidProfile.State(position.rotations, 0.0)
    }

    override fun updateInputs(inputs: IntakeIOInputs) {
        state = profile.calculate(SimConstants.LOOP_TIME, state, goal)
        inputs.armPosition = Rotation2d.fromRotations(state.position)
        inputs.armVelocity = state.velocity.rotationsPerSecond
        // trick homing code
        if (inputs.armPosition == IntakeConstants.STOWED) {
            inputs.armCurrent = IntakeConstants.ARM_DEPLOY_CURRENT_THRESHOLD.amps + 1.amp
        } else {
            inputs.armCurrent = 0.amps
        }
        inputs.rollerVelocity = rollerSpeed.rotationsPerSecond
    }
}
