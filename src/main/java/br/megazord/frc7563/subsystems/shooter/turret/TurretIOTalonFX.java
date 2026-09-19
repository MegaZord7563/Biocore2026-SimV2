// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Turret;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;

public class TurretIOTalonFX implements TurretIO {
    private final TalonFX turretMotor = new TalonFX(Turret.kMotorId, CANBus.roboRIO());
    // create a position closed-loop request, voltage output, slot 0 configs
    private final PositionVoltage turretPID = new PositionVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    /* Keep a brake request so we can disable the motor */
    private final NeutralOut breakeOut = new NeutralOut();

    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    public TurretIOTalonFX() {
        initializeTurretMotor();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs)
    {
        inputs.turretConnected = turretMotor.isConnected();
        inputs.turretPositionRads = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());
        inputs.turretVelocityRadsPerSec = Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble());
        inputs.turretAppliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.turretSupplyCurrentAmps = turretMotor.getSupplyCurrent().getValueAsDouble();
        inputs.turretTempCelsius = turretMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(TurretIOOutputs outputs)
    {
        switch (outputs.mode) {
            case POSITION:
                turretMotor.setControl(turretPID.withPosition(outputs.targetRotation.getRotations()));
                break;
            case VOLTAGE:
                turretMotor.setControl(voltageOut.withOutput(outputs.voltageOut));
                break;
            case COAST:
                turretMotor.setControl(coastOut);
                break;
            case BREAK:
                turretMotor.setControl(breakeOut);
                break;
            default:
                break;
        }
    }

    private void initializeTurretMotor() {
        /* factory default config */
        turretMotor.clearStickyFaults();
        turretMotor.getConfigurator().apply(new TalonFXConfiguration());

        /* set motor neutral mode */
        turretMotor.setNeutralMode(Turret.kMotorNeutralMode);

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
        CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
        currentLimitConfigs.StatorCurrentLimit = Turret.kMotorThresholdCurrent;
        currentLimitConfigs.StatorCurrentLimitEnable = Turret.kMotorEnableCurrentLimit;

        /* Gear Ratio Config */
        toConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        toConfigure.Feedback.SensorToMechanismRatio = Turret.kMotorGearRatio;
        toConfigure.ClosedLoopGeneral.ContinuousWrap = Turret.kContinuousWrap;
        toConfigure.MotorOutput.NeutralMode = Turret.kMotorNeutralMode;

        /* Velocity Cloosed Loop Ramp */
        // toConfigure.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        // TurretConstants.kDriveClosedLoopRamp;

        /* PID Config */ // slot 0
        toConfigure.withSlot0(new Slot0Configs()
                .withKS(Turret.kS)
                .withKV(Turret.kV)
                .withKP(Turret.kP)
                .withKI(Turret.kI)
                .withKD(Turret.kD));

        /* soft limits config */
        toConfigure.SoftwareLimitSwitch.ForwardSoftLimitEnable = Turret.kForwardSoftLimitEnable;
        toConfigure.SoftwareLimitSwitch.ReverseSoftLimitEnable = Turret.kReverseSoftLimitEnable;
        toConfigure.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Turret.kForwardSoftLimitThreshold;
        toConfigure.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Turret.kReverseSoftLimitThreshold;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++) {
            status = turretMotor.getConfigurator().apply(toConfigure);
            if (status.isOK()) {
                break;
            }
        }
        if (!status.isOK()) {
            System.out.println("Could not apply turret configs, error code: " + status.toString());
            DriverStation.reportWarning("Could not apply turret configs, error code: " + status.toString(), false);
        }
    }
}
