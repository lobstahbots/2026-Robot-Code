// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public Rotation2d getHoodPosition() {
        return inputs.hoodPosition;
    }

    public AngularVelocity getHoodVelocity() {
        return inputs.hoodVelocity;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    public Command hoodAngle(Supplier<Rotation2d> angle) {
        return run(() -> io.setHoodPosition(angle.get()));
    }

    public Command operate(Supplier<Rotation2d> hoodAngle, Supplier<AngularVelocity> flywheelVelocity) {
        return run(() -> {
            io.setHoodPosition(hoodAngle.get());
            io.setFlywheelVelocity(flywheelVelocity.get());
        });
    }
}
