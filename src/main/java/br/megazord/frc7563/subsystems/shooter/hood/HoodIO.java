package br.megazord.frc7563.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

    @AutoLog
    public class HoodIOInputs
    {
        public boolean turretConnected = false;
        public double turretPositionRads = 0.0;
        public double turretVelocityRadsPerSec = 0.0;
        public double turretAppliedVoltage = 0.0;
        public double turretSupplyCurrentAmps = 0.0;
        public double turretTempCelsius = 0.0;
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