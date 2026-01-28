package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class Climber extends SubsystemBase {
    private static final double DEPLOY_POSITION = 3.00; // Change this value as needed (Unit is rotations)
    private static final double RETRACT_POSITION = 0.00; // Change this value as needed (Unit is rotations)
    private final SparkMax climberMotor;
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberController;

    public Climber(int ClimberMotorID) {
         //Motor initialization
        this.climberMotor = new SparkMax(ClimberMotorID, MotorType.kBrushless);
        //Encoder initialization
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0.0);

        //Closed loop controller initialization
        climberController = climberMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(20); //20 amps
        config.idleMode(IdleMode.kBrake);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersist); // Note to set PID in this line
    }

    public void gotoPOS(double position) {
        climberController.setReference(position, SparkClosedLoopController.ControlType.kPosition);
    }


    public void deploy() {
        gotoPOS(DEPLOY_POSITION);
    }

    public void retract() {
        gotoPOS(RETRACT_POSITION);
    }
}