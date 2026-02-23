// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SimConstants;

/** Add your docs here. */
public class IntakeIOSimBasic implements IntakeIO {
    private TrapezoidProfile.State state = new TrapezoidProfile.State(IntakeConstants.STOWED.getRotations(), 0);
    private double rollerSpeed = 0.0;
    private TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(IntakeConstants.CRUISE_VELOCITY, IntakeConstants.MAX_ACCELERATION));
    private TrapezoidProfile.State goal = state;

    public void stopArmMotor() {
        goal = new TrapezoidProfile.State(state.position, 0);
    }

    public void setRollerSpeed(double speed) {
        rollerSpeed = 20 * speed;
    }

    public void stopRollerMotor() {
        rollerSpeed = 0.0;
    }

    public void setRollerVoltage(double voltage) {
        setRollerSpeed(voltage / 12.0);
    }

    public void setArmPosition(Rotation2d position) {
        goal = new TrapezoidProfile.State(position.getRotations(), 0);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        state = profile.calculate(SimConstants.LOOP_TIME, state, goal);
        inputs.armPosition = Rotation2d.fromRotations(state.position);
        inputs.armVelocity = state.velocity;
        // trick homing code
        if (inputs.armPosition.equals(IntakeConstants.STOWED)) {
            inputs.armCurrentAmps = IntakeConstants.ARM_DEPLOY_CURRENT_THRESHOLD + 1;
        } else {
            inputs.armCurrentAmps = 0;
        }
        inputs.rollerVelocity = rollerSpeed;
    }
}
