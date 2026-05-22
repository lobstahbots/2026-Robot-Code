package frc.robot.subsystems.shooter

import com.lobstahbots.units.*
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.units.measure.AngularVelocity

import frc.robot.Constants.ShooterConstants
import frc.robot.Constants.SimConstants

class ShooterIOSimBasic : ShooterIO {
    private var state = TrapezoidProfile.State(ShooterConstants.MIN_ANGLE.rotations, 0.0)
    private var flywheelVelocity = 0.0.rpm
    private var profile = TrapezoidProfile(
        TrapezoidProfile.Constraints(
            ShooterConstants.HOOD_CRUISE_VELOCITY,
            ShooterConstants.HOOD_MAX_ACCELERATION
        )
    )
    private var goal = state

    override fun setFlywheelVelocity(velocity: AngularVelocity) {
        flywheelVelocity = velocity
    }

    override fun setHoodPosition(position: Rotation2d) {
        goal = TrapezoidProfile.State(position.rotations, 0.0)
    }

    override fun updateInputs(inputs: ShooterIO.ShooterIOInputs) {
        state = profile.calculate(SimConstants.LOOP_TIME, state, goal)
        inputs.hoodPosition = Rotation2d.fromRotations(state.position)
        inputs.hoodVelocity = state.velocity.rotationsPerSecond
        // trick homing code
        if (inputs.hoodPosition == ShooterConstants.MIN_ANGLE) inputs.hoodCurrent = 26.0
        else inputs.hoodCurrent = 0.0
        inputs.flywheelVelocity = flywheelVelocity
    }
}