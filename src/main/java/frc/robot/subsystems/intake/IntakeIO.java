package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    @AutoLog
    public static class IntakeIOInputs {
        public double armVelocity = 0.0;

        public double armAppliedVoltage = 0.0;

        public double armCurrentAmps = 0.0;

        public double armTempCelcius = 0.0;

        public double rollerVelocity = 0.0;

        public double rollerAppliedVoltage = 0.0;

        public double rollerCurrentAmps = 0.0;

        public double rollerTempCelcius = 0.0;
    }

    public void updateInputs(IntakeIOInputs inputs); 

    public void stopArmMotor ();

    public void stopRollerMotor ();

    public void setArmVoltage (double volts);

    public void setRollerVoltage(double volts);

    public void setArmSpeed(double speed);

    public void setRollerSpeed(double speed);

    public void setArmIdleMode(boolean isBrake);

    public void setRollerIdleMode(boolean isBrake);

    public default void periodic() {};

}
