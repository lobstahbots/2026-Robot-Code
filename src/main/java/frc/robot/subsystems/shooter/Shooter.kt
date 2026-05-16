// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.ShotData;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private AngularVelocity setpoint = RPM.of(0);
    private final AngularVelocity zero = RPM.zero();

    public Trigger atSpeed;

    public Shooter(ShooterIO io) {
        this.io = io;
        atSpeed = new Trigger(() -> inputs.getFlywheelVelocity().isNear(setpoint, ShooterConstants.TOLERANCE)
                && !setpoint.isNear(zero, ShooterConstants.TOLERANCE)).debounce(0.1);
    }

    /**
     * Get the hood position of this shooter.
     * 
     * @return The current hood position
     */
    public Rotation2d getHoodPosition() {
        return inputs.getHoodPosition();
    }

    /**
     * Get the current flywheel velocity
     * 
     * @return The current flywheel velocity.
     */
    public AngularVelocity getFlywheelVelocity() {
        return inputs.getFlywheelVelocity();
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

    public Command velocity(Supplier<AngularVelocity> velocity) {
        return runEnd(() -> io.setFlywheelVelocity(velocity.get()),
                () -> io.setFlywheelVelocity(setpoint = RotationsPerSecond.of(0)));
    }

    public Command voltage(double voltage) {
        return startEnd(() -> io.setFlywheelVoltage(voltage), () -> io.setFlywheelVoltage(0));
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
        return runEnd(() -> {
            io.setHoodPosition(hoodAngle.get());
            io.setFlywheelVelocity(setpoint = flywheelVelocity.get());
        }, () -> {
            io.setHoodPosition(inputs.getHoodPosition());
            io.setFlywheelVoltage(0);
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
        return operate(() -> shotData.get().hoodPosition(), () -> setpoint = shotData.get().flywheelVelocity());
    }

    public Command zero() {
        return runEnd(() -> io.setHoodVoltage(-10), () -> io.setHoodVoltage(0))
                .raceWith(new WaitCommand(0.1).andThen(Commands.idle().until(() -> inputs.getHoodCurrent() > 25)))
                .andThen(Commands.waitSeconds(0.05).andThen(() -> io.resetEncoder(ShooterConstants.MIN_ANGLE)));
    }
}
