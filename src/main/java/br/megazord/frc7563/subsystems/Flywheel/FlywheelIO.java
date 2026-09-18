// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.Flywheel;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface FlywheelIO {

    @AutoLog
    public class FlywheelIOInputs {
        public boolean leaderConnected = false;
        public double leaderPositionRads = 0.0;
        public double leaderVelocityRadsPerSec = 0.0;
        public double leaderAppliedVoltage = 0.0;
        public double leaderSupplyVoltage = 0.0;
        public double leaderSupplyCurrentAmps = 0.0;
        public double leaderTempCelsius = 0.0;

        public boolean followerConnected = false;
        public double followerPositionRads = 0.0;
        public double followerVelocityRadsPerSec = 0.0;
        public double followerAppliedVoltage = 0.0;
        public double followerSupplyVoltage = 0.0;
        public double followerSupplyCurrentAmps = 0.0;
        public double followerTempCelsius = 0.0;
    }

    public static enum FlywheelIOOutputMode {
        COAST,
        BRAKE,
        VELOCITY,
        VOLTAGE,
        CHARACTERIZE
    }

    public class FlywheelIOOutputs {
        public FlywheelIOOutputMode mode = FlywheelIOOutputMode.COAST;
        public double velocityRadsPerSec = 0.0;
        public double voltage = 0.0;

    }

    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void applyOutputs(FlywheelIOOutputs outputs) {}
}
