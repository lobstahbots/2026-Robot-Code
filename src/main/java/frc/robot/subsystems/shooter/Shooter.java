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
import frc.robot.util.math.ShotData;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    /**
     * Get the hood position of this shooter.
     * 
     * @return The current hood position
     */
    public Rotation2d getHoodPosition() {
        return inputs.hoodPosition;
    }

    /**
     * Get the current flywheel velocity
     * 
     * @return The current flywheel velocity.
     */
    public AngularVelocity getFlywheelVelocity() {
        return inputs.hoodVelocity;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }

    /**
     * Constructs a command which keeps the hood at an angle specified by a
     * supplier.
     * 
     * @param angle A supplier for the angle, in the form of a {@link Rotation2d}.
     * @return The constructed command.
     */
    public Command hoodAngle(Supplier<Rotation2d> angle) {
        return run(() -> io.setHoodPosition(angle.get()));
    }

    /**
     * Constructs a command which keeps the hood at an angle specified by a supplier
     * and the flywheel at a velocity specified by a supplier.
     * 
     * @param hoodAngle        A supplier for the hood angle as a
     *                         {@link Rotation2d}.
     * @param flywheelVelocity A supplier for the rotational velocity of the
     *                         flywheel as a {@link AngularVelocity} Measure object.
     * @return The constructed command.
     */
    public Command operate(Supplier<Rotation2d> hoodAngle, Supplier<AngularVelocity> flywheelVelocity) {
        return run(() -> {
            io.setHoodPosition(hoodAngle.get());
            io.setFlywheelVelocity(flywheelVelocity.get());
        });
    }

    /**
     * Constructs a command which keeps the hood at an angle and the flywheel at a
     * velocity specified by a supplier.
     * 
     * @param shotData The supplier for the data for the shot, which supplies
     *                 {@link ShotData} objects.
     * @return The constructed command.
     * @see ShotData
     */
    public Command operate(Supplier<ShotData> shotData) {
        return operate(() -> shotData.get().hoodPosition(), () -> shotData.get().flywheelVelocity());
    }
}
