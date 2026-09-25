// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/** Real IO for the indexer mechanism using CTRE Phoenix 6 TalonFX with VelocityVoltage. */
public class IndexerIOTalonFX implements IndexerIO {
    private final TalonFX motorIndexer = new TalonFX(IndexerConstants.motorIndexerPort, CANBus.roboRIO());

    // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage indexerPID = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    /* Keep a brake request so we can disable the motor */
    private final NeutralOut breakOut = new NeutralOut();

    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    public IndexerIOTalonFX() {
        initializeMotor();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.indexerConnected = motorIndexer.isConnected();
        inputs.indexerPositionRads = Units.rotationsToRadians(motorIndexer.getPosition().getValueAsDouble());
        inputs.indexerVelocityRadsPerSec = Units.rotationsToRadians(motorIndexer.getVelocity().getValueAsDouble());
        inputs.indexerAppliedVoltage = motorIndexer.getMotorVoltage().getValueAsDouble();
        inputs.indexerSupplyVoltage = motorIndexer.getSupplyVoltage().getValueAsDouble();
        inputs.indexerSupplyCurrentAmps = motorIndexer.getSupplyCurrent().getValueAsDouble();
        inputs.indexerTempCelsius = motorIndexer.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(IndexerIOOutputs outputs) {
        switch (outputs.mode) {
            case VELOCITY:
                motorIndexer.setControl(indexerPID.withVelocity(Units.radiansToRotations(outputs.velocityRadsPerSec)));
                break;
            case VOLTAGE:
                motorIndexer.setControl(voltageOut.withOutput(outputs.voltageOut));
                break;
            case BREAK:
                motorIndexer.setControl(breakOut);
                break;
            case COAST:
                motorIndexer.setControl(coastOut);
                break;
            case CHARACTERIZE:
                // Not implemented yet.
                motorIndexer.setControl(coastOut);
                break;
            default:
                break;
        }
    }

    private void initializeMotor() 
    {
        motorIndexer.clearStickyFaults();
        motorIndexer.getConfigurator().apply(new TalonFXConfiguration());

        motorIndexer.setNeutralMode(IndexerConstants.kMotorNeutralMode);

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration();
            
        toConfigure
        //---------------Current Limits------------------
        .withCurrentLimits(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(IndexerConstants.kMotorThresholdCurrent)
            .withStatorCurrentLimitEnable(IndexerConstants.kMotorEnableCurrentLimit))
            
        ///* For validation  - voltage compensation configs MNL 04/24/2026
        .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(12.0)
                        .withPeakReverseVoltage(-12.0)) 
        //--------------Motor Configs-------------------
        .withMotorOutput(new MotorOutputConfigs()
                        .withInverted(InvertedValue.Clockwise_Positive)
                        .withNeutralMode(IndexerConstants.kMotorNeutralMode))
        
        //--------------Closed Loop General--------------
        .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                              .withContinuousWrap(IndexerConstants.kContinuousWrap))
          
        //--------------Feedback Sensor-----------------
        .withFeedback(new FeedbackConfigs()
                      .withSensorToMechanismRatio(IndexerConstants.kMotorGearRatio))
        
        //--------------Closed Loop Ramp-------------
        .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                            .withVoltageClosedLoopRampPeriod(0))
                            
        //-----------PID Slot 0------------------
        .withSlot0(new Slot0Configs()
                    .withKS(IndexerConstants.kSlot0kS)
                    .withKV(IndexerConstants.kSlot0kV)
                    .withKP(IndexerConstants.kSlot0kP)
                    .withKI(IndexerConstants.kSlot0kI)
                    .withKD(IndexerConstants.kSlot0kD));

        /* apply configuration to motor */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; i++)
        {
            status = motorIndexer.getConfigurator().apply(toConfigure);
            if (status.isOK())
            {
                break;
            }
        }
        if (!status.isOK())
        {
            DriverStation.reportWarning("Failed to apply indexer motor configuration, error: " + status.toString(), false);
        }
    }
}
