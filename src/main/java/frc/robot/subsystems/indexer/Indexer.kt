// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.indexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Indexer
/** Creates a new Indexer.  */(private val io: IndexerIO) : SubsystemBase() {
    private val inputs = IndexerIOInputsAutoLogged()

    fun setFeederSpeed(speed: Double) = io.setFeederSpeed(speed)

    fun stopIndexerMotor() = io.stopIndexer()

    fun setIdleMode(isBrake: Boolean) = io.setIdleMode(isBrake)

    var indexerSpeed: Double
        get() = inputs.indexerVelocity.baseUnitMagnitude()
        set(speed) {
            io.setIndexerSpeed(speed)
        }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("Indexer", inputs)
    }

    fun spindex(): Command = runEnd({
        io.setIndexerSpeed(0.5)
        io.setFeederSpeed(0.75)
    }, io::stopIndexer)

    fun feed(): Command = runEnd({ io.setFeederSpeed(0.75) }, io::stopIndexer)
}
