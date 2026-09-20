// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter.Flywheel;

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

import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Flywheel;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class FlywheelIOTalonFX implements FlywheelIO {
    private final TalonFX leaderMotor = new TalonFX(Flywheel.kLeaderMotorId, CANBus.roboRIO());
    private final TalonFX followerMotor = new TalonFX(Flywheel.kFollowerMotorId, CANBus.roboRIO());

    // create a position closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage flywheelPID = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    /* Keep a brake request so we can disable the motor */
    private final NeutralOut breakOut = new NeutralOut();

    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    public FlywheelIOTalonFX() {
        initializeMotors();

        // make the second TalonFX follow the shooter wheel motor
        followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leaderConnected = leaderMotor.isConnected();
        inputs.followerConnected = followerMotor.isConnected();

        inputs.leaderAppliedVoltage = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.leaderVelocityRadsPerSec = Units.rotationsToRadians(leaderMotor.getVelocity().getValueAsDouble());
        inputs.leaderPositionRads = Units.rotationsToRadians(leaderMotor.getPosition().getValueAsDouble());
        inputs.leaderSupplyCurrentAmps = leaderMotor.getSupplyCurrent().getValueAsDouble();
        inputs.leaderTempCelsius = leaderMotor.getDeviceTemp().getValueAsDouble();
        inputs.followerAppliedVoltage = followerMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerVelocityRadsPerSec = Units.rotationsToRadians(followerMotor.getVelocity().getValueAsDouble());
        inputs.followerPositionRads = Units.rotationsToRadians(followerMotor.getPosition().getValueAsDouble());
        inputs.followerSupplyCurrentAmps = followerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.followerTempCelsius = followerMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        switch (outputs.mode) {
            case VELOCITY:
                leaderMotor.setControl(flywheelPID.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
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
        leaderMotor.setNeutralMode(Flywheel.kMotorNeutralMode);
        followerMotor.setNeutralMode(Flywheel.kMotorNeutralMode);

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimit = Flywheel.kMotorThresholdCurrent;
        currentLimitConfigs.StatorCurrentLimitEnable = Flywheel.kMotorEnableCurrentLimit;
        currentLimitConfigs.SupplyCurrentLimit = Flywheel.kMotorSupplyCurrent;

        /* Gear Ratio Config */
        toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        toConfigure.MotorOutput.NeutralMode = Flywheel.kMotorNeutralMode;
        toConfigure.ClosedLoopGeneral.ContinuousWrap = Flywheel.kContinuousWrap;
        toConfigure.Feedback.SensorToMechanismRatio = 1;

        followerMotor.getConfigurator().apply(toConfigure);

        toConfigure.Slot0.kS = Flywheel.kSlot0kS;
        toConfigure.Slot0.kV = Flywheel.kSlot0kV;
        toConfigure.Slot0.kP = Flywheel.kSlot0kP;
        toConfigure.Slot0.kI = Flywheel.kSlot0kI;
        toConfigure.Slot0.kD = Flywheel.kSlot0kD;

        // toConfigure.Voltage.withPeakForwardVoltage(Volts.of(8))
        // .withPeakReverseVoltage(Volts.of(-8));

        toConfigure.Slot1.kS = Flywheel.kSlot1kS;
        toConfigure.Slot1.kV = Flywheel.kSlot1kV;
        toConfigure.Slot1.kP = Flywheel.kSlot1kP;
        toConfigure.Slot1.kI = Flywheel.kSlot1kI;
        toConfigure.Slot1.kD = Flywheel.kSlot1kD;

        // toConfigure.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
        // .withPeakReverseTorqueCurrent(Amps.of(-40));

        /* apply configuration to motor */
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
                    "Failed to apply shooter flywheel leader motor configuration, error: " + statusLeader.toString(),
                    false);
        }
    }
}