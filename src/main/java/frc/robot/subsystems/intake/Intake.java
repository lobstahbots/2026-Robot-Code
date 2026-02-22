// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeIO io;
    private final LoggedMechanism2d mech2d = new LoggedMechanism2d(Feet.of(3), Feet.of(2));

    private final LoggedMechanismRoot2d root = mech2d.getRoot("Intake",
            Feet.of(1.5).plus(Inches.of(6.5)).baseUnitMagnitude(), Inches.of(8).baseUnitMagnitude());
    private final LoggedMechanismLigament2d arm2d = root
            .append(new LoggedMechanismLigament2d("Arm", Inches.of(12), Degrees.of(0)));

    private boolean hasZeroed = false;

    /** Creates a new Intake. */
    public Intake(IntakeIO io) {
        this.io = io;
    }

    public void setArmIdleMode(boolean isBrake) {
        io.setArmIdleMode(isBrake);
    }

    public void setRollerIdleMode(boolean isBrake) {
        io.setRollerIdleMode(isBrake);
    }

    public double getRollerVelocity() {
        return inputs.rollerVelocity;
    }

    public Rotation2d getPosition() {
        return inputs.armPosition;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        arm2d.setAngle(getPosition());
        Logger.recordOutput("IntakeMech2d", mech2d);
    }

    public Command spin() {
        return runEnd(() -> io.setRollerSpeed(1), io::stopRollerMotor);
    }

    private Command setPosition(Rotation2d position) {
        return startEnd(() -> io.setArmPosition(position), io::stopArmMotor).until(() -> Math
                .abs(position.minus(getPosition()).getRotations()) <= IntakeConstants.MAX_ERROR.getRotations());
    }

    public Command deploy() {
        return setPosition(IntakeConstants.DEPLOYED);
    }

    public Command stow() {
        return setPosition(IntakeConstants.STOWED);
    }

    public Command home() {
        return startEnd(() -> io.setArmVoltage(-6), () -> {
            io.stopArmMotor();
            hasZeroed = true;
            io.resetEncoder(IntakeConstants.STOWED);
        }).until(() -> inputs.armCurrentAmps >= IntakeConstants.ARM_DEPLOY_CURRENT_THRESHOLD);
    }
}
