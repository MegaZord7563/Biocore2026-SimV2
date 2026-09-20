package br.megazord.frc7563.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public class HoodIOInputs
    {
        public boolean hoodConnected = false;
        public double hoodPositionRads = 0.0;
        public double hoodVelocityRadsPerSec = 0.0;
        public double hoodAppliedVoltage = 0.0;
        public double hoodSupplyCurrentAmps = 0.0;
        public double hoodTempCelsius = 0.0;
    }

    public enum HoodIOOutputsMode
    {
        POSITION,
        VOLTAGE,
        BREAK,
        COAST;
    }

    public class HoodIOOutputs
    {
        public HoodIOOutputsMode mode = HoodIOOutputsMode.COAST;
        public double targetPositionRads = 0.0;
        public double voltageOut = 0.0;
    }

    public default void updateInputs(HoodIOInputs inputs) {}
    public default void applyOutputs(HoodIOOutputs outputs) {}
} 