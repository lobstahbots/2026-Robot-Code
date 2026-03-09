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

import frc.robot.Constants.IndexerConstants;

public class IndexerIOSparkMax implements IndexerIO {

    private final SparkMax indexerMotor;
    private final SparkMax feederMotor;
    private final RelativeEncoder encoder;
    private final RelativeEncoder feederEncoder;
    /** Creates a new Indexer. */
    public IndexerIOSparkMax(int indexerMotorID, int feederMotorId) {
        this.indexerMotor = new SparkMax(indexerMotorID, MotorType.kBrushless);
        this.feederMotor = new SparkMax(feederMotorId, MotorType.kBrushless);
    
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(IndexerConstants.CURRENT_LIMIT).idleMode(IdleMode.kBrake).inverted(true).encoder.velocityConversionFactor(3);

        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = indexerMotor.getEncoder();
        feederEncoder = feederMotor.getEncoder();
    }

    public void setIndexerSpeed(double speed) {
        indexerMotor.set(speed);
        feederMotor.set(speed);
    }

    public void stopIndexer() {
        indexerMotor.stopMotor();
        feederMotor.stopMotor();
    }

    public void setIdleMode(boolean isBrake) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(isBrake ? IdleMode.kBrake : IdleMode.kCoast);
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerVelocity = encoder.getVelocity();
        inputs.indexerCurrentAmps = indexerMotor.getOutputCurrent();
        inputs.indexerAppliedVoltage = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
        inputs.indexerTempCelcius = indexerMotor.getMotorTemperature();
        inputs.feederVelocity = feederEncoder.getVelocity();
        inputs.feederCurrentAmps = feederMotor.getOutputCurrent();
        inputs.feederAppliedVoltage = feederMotor.getAppliedOutput() * feederMotor.getBusVoltage();
        inputs.feederTempCelcius = feederMotor.getMotorTemperature();
    }
}
