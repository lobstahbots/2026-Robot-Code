package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs {

        /**
         * Velocity of intake arm in rotations/second
         */
        public double armVelocity = 0.0;

        /**
         * Voltage of intake arm motor in volts
         */
        public double armAppliedVoltage = 0.0;

        /**
         * Current of intake arm motor in amperes
         */
        public double armCurrentAmps = 0.0;

        /**
         * Temperature of intake arm motor in celcius
         */
        public double armTempCelcius = 0.0;

        /**
         * Position of intake arm in rotations
         */
        public Rotation2d armPosition = Rotation2d.kZero;

        /**
         * Velocity of roller motor in rotations/second
         */
        public double rollerVelocity = 0.0;

        /**
         * Voltage of roller motor in volts
         */
        public double rollerAppliedVoltage = 0.0;

        /**
         * Current of roller motor in amperes
         */
        public double rollerCurrentAmps = 0.0;

        /**
         * Temperature of roller motor in celcius
         */
        public double rollerTempCelcius = 0.0;

        public boolean isDeployed = false;
    }

    public void updateInputs(IntakeIOInputs inputs);

    public void stopArmMotor();

    public void stopRollerMotor();

    public void setArmVoltage(double volts);

    public void setRollerVoltage(double volts);

    public void setArmPosition(Rotation2d position);

    public void setRollerSpeed(double speed);

    public void setArmIdleMode(boolean isBrake);

    public void setRollerIdleMode(boolean isBrake);

    public void resetEncoder(Rotation2d position);

    public default void periodic() {};

}
