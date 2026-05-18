// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.indexer

import com.lobstahbots.units.*
import com.revrobotics.PersistMode
import com.revrobotics.RelativeEncoder
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.SparkMax
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import frc.robot.Constants.*
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs


class IndexerIOSparkMax(indexerMotorID: Int, feederMotorId: Int) : IndexerIO {
    private val indexerMotor: SparkMax = SparkMax(indexerMotorID, SparkLowLevel.MotorType.kBrushless)
    private val feederMotor: SparkMax = SparkMax(feederMotorId, SparkLowLevel.MotorType.kBrushless)
    private val encoder: RelativeEncoder
    private val feederEncoder: RelativeEncoder

    /** Creates a new Indexer.  */
    init {
        val config = SparkMaxConfig()
        config.smartCurrentLimit(IndexerConstants.SPINDEXER_CURRENT_LIMIT).idleMode(IdleMode.kBrake)
            .inverted(true).encoder.velocityConversionFactor(3.0)

        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        config.smartCurrentLimit(IndexerConstants.FEEDER_MOTOR_CURRENT_LIMIT).inverted(false)
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        encoder = indexerMotor.encoder
        feederEncoder = feederMotor.encoder
    }

    override fun setIndexerSpeed(speed: Double) = indexerMotor.set(speed)

    override fun setFeederSpeed(speed: Double) = feederMotor.set(speed)

    override fun stopIndexer() {
        indexerMotor.stopMotor()
        feederMotor.stopMotor()
    }

    override fun setIdleMode(isBrake: Boolean) {
        val config = SparkMaxConfig()
        config.idleMode(if (isBrake) IdleMode.kBrake else IdleMode.kCoast)
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun updateInputs(inputs: IndexerIOInputs) {
        inputs.indexerVelocity = encoder.velocity.rpm
        inputs.indexerCurrent = indexerMotor.outputCurrent.amps
        inputs.indexerAppliedVoltage = indexerMotor.appliedOutput.value * indexerMotor.busVoltage.volts
        inputs.indexerTemp = indexerMotor.motorTemperature.celsius
        inputs.feederVelocity = feederEncoder.velocity.rpm
        inputs.feederCurrent = feederMotor.outputCurrent.amps
        inputs.feederAppliedVoltage = feederMotor.appliedOutput.value * feederMotor.busVoltage.volts
        inputs.feederTemp = feederMotor.motorTemperature.celsius
    }
}
