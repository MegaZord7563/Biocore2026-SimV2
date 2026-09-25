// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.feeder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/** Real IO for the feeder (leader + follower rollers, mechanically aligned). */
public class FeederIOTalonFX implements FeederIO {
    private final TalonFX leaderMotor = new TalonFX(FeederConstants.kLeaderMotorId, CANBus.roboRIO());
    private final TalonFX followerMotor = new TalonFX(FeederConstants.kFollowerMotorId, CANBus.roboRIO());

    // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage feederPID = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    /* Keep a brake request so we can disable the motors */
    private final NeutralOut breakOut = new NeutralOut();

    /* Keep a coast request so we can disable the motors */
    private final CoastOut coastOut = new CoastOut();

    public FeederIOTalonFX() {
        initializeMotors();

        // the motors are mechanically aligned (same shaft direction), so the
        // follower spins the same way as the leader instead of opposing it
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.leaderConnected = leaderMotor.isConnected();
        inputs.leaderPositionRads = Units.rotationsToRadians(leaderMotor.getPosition().getValueAsDouble());
        inputs.leaderVelocityRadsPerSec = Units.rotationsToRadians(leaderMotor.getVelocity().getValueAsDouble());
        inputs.leaderAppliedVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.leaderSupplyVoltage = leaderMotor.getSupplyVoltage().getValueAsDouble();
        inputs.leaderSupplyCurrentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
        inputs.leaderTempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();

        inputs.followerConnected = followerMotor.isConnected();
        inputs.followerPositionRads = Units.rotationsToRadians(followerMotor.getPosition().getValueAsDouble());
        inputs.followerVelocityRadsPerSec = Units.rotationsToRadians(followerMotor.getVelocity().getValueAsDouble());
        inputs.followerAppliedVoltage = followerMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerSupplyCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.followerTempCelsius = followerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(FeederIOOutputs outputs) {
        switch (outputs.mode) {
            case VELOCITY:
                leaderMotor.setControl(feederPID.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
                break;
            case VOLTAGE:
                leaderMotor.setControl(voltageOut.withOutput(outputs.voltageOut));
                break;
            case COAST:
                leaderMotor.setControl(coastOut);
                break;
            case BREAK:
                leaderMotor.setControl(breakOut);
                break;
            case CHARACTERIZE:
                // Not implemented yet.
                leaderMotor.setControl(coastOut);
                break;
            default:
                break;
        }
    }

    private void initializeMotors() {
        /* factory default config */
        leaderMotor.clearStickyFaults();
        followerMotor.clearStickyFaults();

        leaderMotor.getConfigurator().apply(new TalonFXConfiguration());
        followerMotor.getConfigurator().apply(new TalonFXConfiguration());

        /* set motor neutral mode */
        leaderMotor.setNeutralMode(FeederConstants.kMotorNeutralMode);
        followerMotor.setNeutralMode(FeederConstants.kMotorNeutralMode);

        TalonFXConfiguration toConfigure = new TalonFXConfiguration();

        /* current limits */
        CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimit = FeederConstants.kMotorThresholdCurrent;
        currentLimitConfigs.StatorCurrentLimitEnable = FeederConstants.kMotorEnableCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimit = FeederConstants.kMotorSupplyCurrent;
        currentLimitConfigs.SupplyCurrentLimitEnable = FeederConstants.kMotorEnableCurrentLimit;

        /* gear ratio / motor output config */
        toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        toConfigure.MotorOutput.NeutralMode = FeederConstants.kMotorNeutralMode;
        toConfigure.ClosedLoopGeneral.ContinuousWrap = FeederConstants.kContinuousWrap;
        toConfigure.Feedback.SensorToMechanismRatio = FeederConstants.kMotorGearRatio;

        /* PID slot 0 */
        toConfigure.Slot0.kS = FeederConstants.kSlot0kS;
        toConfigure.Slot0.kV = FeederConstants.kSlot0kV;
        toConfigure.Slot0.kP = FeederConstants.kSlot0kP;
        toConfigure.Slot0.kI = FeederConstants.kSlot0kI;
        toConfigure.Slot0.kD = FeederConstants.kSlot0kD;

        /* apply configuration to leader; follower gets the same config, then
         * has its Follower control request re-applied by the constructor */
        StatusCode statusLeader = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            statusLeader = leaderMotor.getConfigurator().apply(toConfigure);
            followerMotor.getConfigurator().apply(toConfigure);
            if (statusLeader.isOK()) {
                break;
            }
        }
        if (!statusLeader.isOK()) {
            DriverStation.reportWarning(
                    "Failed to apply feeder leader motor configuration, error: " + statusLeader.toString(),
                    false);
        }
    }
}
