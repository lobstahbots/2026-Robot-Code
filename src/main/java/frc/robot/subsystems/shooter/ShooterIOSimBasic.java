// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SimConstants;

/** Add your docs here. */
public class ShooterIOSimBasic implements ShooterIO {
    private TrapezoidProfile.State state = new TrapezoidProfile.State(ShooterConstants.MIN_ANGLE.getRotations(), 0);
    private double flywheelVelocity = 0.0;
    private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
            ShooterConstants.HOOD_CRUISE_VELOCITY, ShooterConstants.HOOD_MAX_ACCELERATION));
    private TrapezoidProfile.State goal = state;

    public void stopArmMotor() {
        goal = new TrapezoidProfile.State(state.position, 0);
    }

    public void setFlywheelVelocity(double speed) {
        flywheelVelocity = 20 * speed;
    }

    public void stopFlywheelMotor() {
        flywheelVelocity = 0.0;
    }

    public void setFlywheelVoltage(double voltage) {
        setFlywheelVelocity(voltage / 12.0);
    }

    public void setHoodPosition(Rotation2d position) {
        goal = new TrapezoidProfile.State(position.getRotations(), 0);
    }

    public void updateInputs(ShooterIOInputs inputs) {
        state = profile.calculate(SimConstants.LOOP_TIME, state, goal);
        inputs.hoodPosition = Rotation2d.fromRotations(state.position);
        inputs.hoodVelocity = RotationsPerSecond.of(state.velocity);
        // trick homing code
        if (inputs.hoodPosition == ShooterConstants.MIN_ANGLE) {
            inputs.hoodCurrent = 11;
        } else {
            inputs.hoodCurrent = 0;
        }
        inputs.flywheelVelocity = RotationsPerSecond.of(flywheelVelocity);
    }
}
