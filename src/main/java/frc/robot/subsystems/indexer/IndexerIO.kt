package frc.robot.subsystems.indexer

import com.lobstahbots.junction.AutoLogKt
import com.lobstahbots.units.*

interface IndexerIO {
    @AutoLogKt
    open class IndexerIOInputs {
        /**
         * Indexer wheel velocity in rotations/second
         */
        var indexerVelocity = 0.rotationsPerSecond

        /**
         * Indexer motor current in amperes
         */
        var indexerCurrent = 0.amps

        /**
         * Indexer motor applied voltage in volts
         */
        var indexerAppliedVoltage = 0.volts

        /**
         * Indexer motor temperature in celcius
         */
        var indexerTemp = 0.celsius

        /**
         * Indexer wheel velocity in rotations/second
         */
        var feederVelocity = 0.rotationsPerSecond

        /**
         * Indexer motor current in amperes
         */
        var feederCurrent = 0.amps

        /**
         * Indexer motor applied voltage in volts
         */
        var feederAppliedVoltage = 0.volts

        /**
         * Indexer motor temperature
         */
        var feederTemp = 0.celsius
    }

    fun updateInputs(inputs: IndexerIOInputs) {}

    fun setIndexerSpeed(speed: Double) {}

    fun setFeederSpeed(speed: Double) {}

    fun stopIndexer() {}

    fun setIdleMode(isBrake: Boolean) {}

    fun periodic() {}
}
