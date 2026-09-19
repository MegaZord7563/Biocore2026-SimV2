package br.megazord.frc7563.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs
    {
        public boolean turretConnected = false;
        public double turretPositionRads = 0.0;
        public double turretVelocityRadsPerSec = 0.0;
        public double turretAppliedVoltage = 0.0;
        public double turretSupplyCurrentAmps = 0.0;
        public double turretTempCelsius = 0.0;
    }

    public enum TurretIOMode
    {
        POSITION,
        VOLTAGE,
        BREAK,
        COAST;
    }

    public class TurretIOOutputs
    {
        public TurretIOMode mode = TurretIOMode.COAST;
        public Rotation2d targetRotation = Rotation2d.kZero;
        public double voltageOut = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}
    public default void applyOutputs(TurretIOOutputs outputs) {}
} 