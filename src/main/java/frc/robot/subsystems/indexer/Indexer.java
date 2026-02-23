// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;


import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {

    private final SparkMax indexerMotor;
    private final RelativeEncoder encoder;
    /** Creates a new Indexer. */
    public Indexer(int indexerMotorID) {
        this.indexerMotor = new SparkMax(indexerMotorID, MotorType.kBrushless);
    
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IndexerConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(false).encoder.velocityConversionFactor(3);

        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = indexerMotor.getEncoder();
    }

    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
    }

    public void setIdleMode(boolean isBrake) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
