// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    /** Creates a new Indexer. */
    public Indexer(IndexerIO io) {
        this.io = io;
    }

    public void setIndexerSpeed(double speed) {
        io.setIndexerSpeed(speed);
    }

    public void stopIndexerMotor() {
        io.stopIndexer();
    }

    public void setIdleMode(boolean isBrake) {
        io.setIdleMode(isBrake);
    }

    public double getIndexerSpeed() {
        return inputs.indexerVelocity;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public Command spin() {
        return runEnd(() -> io.setIndexerSpeed(1), io::stopIndexer);
    }
}
