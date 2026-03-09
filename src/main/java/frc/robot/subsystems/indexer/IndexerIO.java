package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    
    @AutoLog
    public static class IndexerIOInputs {
        /**
         * Indexer wheel velocity in rotations/second
         */

        public double indexerVelocity = 0.0;

        /**
         * Indexer motor current in amperes
         */

        public double indexerCurrentAmps = 0.0;

        /**
         * Indexer motor applied voltage in volts
         */

        public double indexerAppliedVoltage = 0.0;

        /**
         * Indexer motor temperature in celcius
         */

        public double indexerTempCelcius = 0.0;
        /**
         * Indexer wheel velocity in rotations/second
         */

        public double feederVelocity = 0.0;

        /**
         * Indexer motor current in amperes
         */

        public double feederCurrentAmps = 0.0;

        /**
         * Indexer motor applied voltage in volts
         */

        public double feederAppliedVoltage = 0.0;

        /**
         * Indexer motor temperature in celcius
         */

        public double feederTempCelcius = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setIndexerSpeed(double speed) {}

    public default void stopIndexer() {}
    
    public default void setIdleMode(boolean isBrake) {}

    public default void periodic() {}
}
