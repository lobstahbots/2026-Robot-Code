package frc.robot.subsystems.indexer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final SparkMax indexer;
    private int indexerSpeed;

    public Indexer(int inexerMotorID, int indexerSpeed) {
        indexer = new SparkMax(inexerMotorID, MotorType.kBrushless);
        this.indexerSpeed = indexerSpeed;
    }

    public void startMotor() {
        indexer.set(indexerSpeed);
    }

    public void stopMotor() {
        indexer.stopMotor();
    }
}
