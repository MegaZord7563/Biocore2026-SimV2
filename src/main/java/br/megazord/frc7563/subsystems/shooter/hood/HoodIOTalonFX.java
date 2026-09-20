// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter.hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Hood;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.DriverStation;

public class HoodIOTalonFX implements HoodIO {
    private final TalonFX hoodMotor = new TalonFX(Hood.kMotorId, CANBus.roboRIO());

    // create a position closed-loop request, voltage output, slot 0 configs
    private final MotionMagicVoltage hoodPID = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    /* Keep a brake request so we can disable the motor */
    private final NeutralOut breakOut = new NeutralOut();

    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    public HoodIOTalonFX() {
        initializeHoodMotor();
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.hoodConnected = hoodMotor.isConnected();
        inputs.hoodPositionRads = Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble());
        inputs.hoodVelocityRadsPerSec = Units.rotationsToRadians(hoodMotor.getVelocity().getValueAsDouble());
        inputs.hoodAppliedVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.hoodSupplyCurrentAmps = hoodMotor.getSupplyCurrent().getValueAsDouble();
        inputs.hoodTempCelsius = hoodMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(HoodIOOutputs outputs) {
        switch (outputs.mode) {
            case POSITION:
                hoodMotor.setControl(hoodPID.withPosition(Units.radiansToRotations(outputs.targetPositionRads)));
                break;
            case VOLTAGE:
                hoodMotor.setControl(voltageOut.withOutput(outputs.voltageOut));
                break;
            case COAST:
                hoodMotor.setControl(coastOut);
                break;
            case BREAK:
                hoodMotor.setControl(breakOut);
                break;
            default:
                break;
        }
    }

    private void initializeHoodMotor() {
        // factory default config
        hoodMotor.clearStickyFaults();
        hoodMotor.getConfigurator().apply(new TalonFXConfiguration());

        // set motor neutral mode
        hoodMotor.setNeutralMode(Hood.kMotorNeutralMode);

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimit = Hood.kMotorThresholdCurrent;
        currentLimitConfigs.StatorCurrentLimitEnable = Hood.kMotorEnableCurrentLimit;

        /* Gear Ratio Config */
        toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        toConfigure.Feedback.SensorToMechanismRatio = Hood.kMotorGearRatio;
        toConfigure.ClosedLoopGeneral.ContinuousWrap = Hood.kContinuousWrap;
        toConfigure.MotorOutput.NeutralMode = Hood.kMotorNeutralMode;

        /* Velocity Cloosed Loop Ramp */
        // toConfigure.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        // CapoArticulatorConstants.kDriveClosedLoopRamp;

        /* PID Config */ // slot 0
        toConfigure.withSlot0(new Slot0Configs()
                .withKS(Hood.kS)
                .withKV(Hood.kV)
                .withKP(Hood.kP)
                .withKI(Hood.kI)
                .withKD(Hood.kD));

        toConfigure.SoftwareLimitSwitch.ForwardSoftLimitEnable = Hood.kForwardSoftLimitEnable;
        toConfigure.SoftwareLimitSwitch.ReverseSoftLimitEnable = Hood.kReverseSoftLimitEnable;
        toConfigure.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Hood.kForwardSoftLimitThreshold;
        toConfigure.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Hood.kReverseSoftLimitThreshold;

        MotionMagicConfigs mm = toConfigure.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(20))
                .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10))
                .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(100));

        /* Speed up signals to an appropriate rate */
        // BaseStatusSignal.setUpdateFrequencyForAll(250, driveMotor.getPosition(),
        // driveMotor.getVelocity());

        /* Apply configuration */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = hoodMotor.getConfigurator().apply(toConfigure);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply hood configs, error code: " + status.toString());
            DriverStation.reportWarning("Could not apply hood configs, error code: " + status.toString(),
                    false);
        }
    }
}