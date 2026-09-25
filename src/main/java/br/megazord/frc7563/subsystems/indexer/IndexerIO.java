// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public class IndexerIOInputs {
        public boolean indexerConnected = false;
        public double indexerPositionRads = 0.0;
        public double indexerVelocityRadsPerSec = 0.0;
        public double indexerAppliedVoltage = 0.0;
        public double indexerSupplyVoltage = 0.0;
        public double indexerSupplyCurrentAmps = 0.0;
        public double indexerTempCelsius = 0.0;
    }

    public static enum IndexerIOOutputMode {
        COAST,
        BREAK,
        VELOCITY,
        VOLTAGE,
        CHARACTERIZE
    }

    public class IndexerIOOutputs {
        public IndexerIOOutputMode mode = IndexerIOOutputMode.BREAK;
        public double velocityRadsPerSec = 0.0;
        public double voltageOut = 0.0;
        public double CharacterizationOutput = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void applyOutputs(IndexerIOOutputs outputs) {}
}
