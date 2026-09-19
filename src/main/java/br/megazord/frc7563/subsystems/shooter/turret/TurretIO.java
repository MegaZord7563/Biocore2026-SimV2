package br.megazord.frc7563.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs
    {

    }

    public class TurretIOOutputs
    {

    }

    public default void updateInputs(TurretIOInputs inputs) {}
    public default void applyOutputs(TurretIOOutputs outputs) {}
} 