package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0);
        public double[] flywheelAppliedVoltages = new double[3];
        public double[] flywheelCurrents = new double[3];
        public double[] flywheelTemperatures = new double[3];

        public Rotation2d hoodPosition = Rotation2d.kZero;
        public double hoodAppliedVoltage = 0.0;
        public double hoodCurrent = 0.0;
        public double hoodTemperature = 0.0;
        public AngularVelocity hoodVelocity = RotationsPerSecond.of(0);
    }

    public default void setFlywheelVoltage(double voltage) {}

    public default void setHoodVoltage(double voltage) {}

    public default void setFlywheelVelocity(AngularVelocity velocity) {}

    public default void setHoodPosition(Rotation2d position) {}

    public default void updateInputs(ShooterIOInputs inputs) {}
}
