// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import br.megazord.frc7563.subsystems.intake.IntakeConstants.Pivot;
import br.megazord.frc7563.subsystems.intake.IntakeConstants.Rollers;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import static edu.wpi.first.units.Units.*;

/** Simulated IO for the intake (pivot + rollers). */
public class IntakeIOTalonFX implements IntakeIO 
{
    private final TalonFX pivotMotor = new TalonFX(Pivot.kMotorId, CANBus.roboRIO());
    private final TalonFX rollersMotor = new TalonFX(Rollers.kMotorId, CANBus.roboRIO());

    // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage rollersPID = new VelocityVoltage(0).withSlot(0);
    // create a position closed-loop request, voltage output, slot 0 configs
    private final PositionVoltage pivotPID = new PositionVoltage(0).withSlot(0);
    /* Keep a brake request so we can disable the motor */
    private final NeutralOut breakOut = new NeutralOut();
    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    public IntakeIOTalonFX() 
    {
        initializeRollersMotor();
        initializePivotMotor();
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.pivotConnected = pivotMotor.isConnected();
        inputs.pivotPositionRads = Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble());
        inputs.pivotVelocityRadsPerSec = Units.rotationsToRadians(pivotMotor.getVelocity().getValueAsDouble());
        inputs.pivotAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.pivotTempCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();

        inputs.rollersConnected = rollersMotor.isConnected();
        inputs.rollerPositionRads = Units.rotationsToRadians(rollersMotor.getPosition().getValueAsDouble());
        inputs.rollersVelocityRadsPerSec = Units.rotationsToRadians(rollersMotor.getVelocity().getValueAsDouble());
        inputs.rollersAppliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollersSupplyCurrentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rollersTempCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(IntakeIOOutputs outputs) 
    {
        switch (outputs.mode) {
            case COAST:
                pivotMotor.setControl(coastOut);
                rollersMotor.setControl(coastOut);
                break;
            case BREAK:
                pivotMotor.setControl(breakOut);
                rollersMotor.setControl(breakOut);
                break;
            case RUN:
                rollersMotor.setControl(rollersPID.withVelocity(Units.radiansToRotations(outputs.rollersSpeedRadPerSec)));
                pivotMotor.setControl(coastOut);
                break;
            case MOVE:
                rollersMotor.setControl(rollersPID.withVelocity(Units.radiansToRotations(outputs.rollersSpeedRadPerSec)));
                pivotMotor.setControl(pivotPID.withPosition(Units.radiansToRotations(outputs.pivotTargetPositionRads)));
                break;
            case CHARACTERIZE:
                // Not implemented yet. 
                pivotMotor.setControl(coastOut);
                rollersMotor.setControl(coastOut);
                break;
            default:
                break;
        }
    }

     private void initializeRollersMotor() 
  {
    /* Factore default */
    rollersMotor.clearStickyFaults();
    rollersMotor.getConfigurator().apply(new TalonFXConfiguration());

    // set motor neutral mode
    rollersMotor.setNeutralMode(Rollers.kMotorNeutralMode);

    /* Configure a stator limit of 80 amps */
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitConfigs = toConfigure.CurrentLimits;
    currentLimitConfigs.StatorCurrentLimit = Rollers.kMotorThresholdCurrent;
    currentLimitConfigs.SupplyCurrentLimit = Rollers.kMotorSupplyCurrent;
    currentLimitConfigs.StatorCurrentLimitEnable = Rollers.kMotorEnableCurrentLimit;

    toConfigure.withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12.0)
                    .withPeakReverseVoltage(-12.0)) ;
    
    /* Gear Ratio Config */
    toConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    toConfigure.Feedback.SensorToMechanismRatio = Rollers.kMotorGearRatio;
    toConfigure.ClosedLoopGeneral.ContinuousWrap = false;
    toConfigure.MotorOutput.NeutralMode = Rollers.kMotorNeutralMode;

    /* PID Config */ // slot 0
    toConfigure.withSlot0(new Slot0Configs()
        .withKS(Rollers.kS)
        .withKV(Rollers.kV)
        .withKP(Rollers.kP)
        .withKI(Rollers.kI)
        .withKD(Rollers.kD));

    toConfigure.withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(RotationsPerSecond.of(10.0))
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10.0))
        .withMotionMagicJerk((RotationsPerSecondPerSecond.per(Seconds)).of(50)));

    /* Apply configuration */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rollersMotor.getConfigurator().apply(toConfigure);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply intake configs, error code: " + status.toString());
      DriverStation.reportWarning("Could not apply intake configs, error code: " + status.toString(), false);
    }

  }

  /**
   * Initialize articulator motor with factory default and apply configurations such as current limits, 
   * PID constants, feedback sensor, and motion magic parameters. This method is called in the constructor 
   * to set up the articulator motor for use.
   */
  private void initializePivotMotor() 
  {
    /* Factory default */
    pivotMotor.clearStickyFaults();
    pivotMotor.getConfigurator().apply(new TalonFXConfiguration());

    // set motor neutral mode
    pivotMotor.setNeutralMode(Pivot.kMotorNeutralMode);

    /* Configure a stator limit of 20 amps */
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();

    toConfigure
    //---------------Current Limits------------------
    .withCurrentLimits(new CurrentLimitsConfigs()
                      .withStatorCurrentLimit(Pivot.kMotorThresholdCurrent)
                      .withSupplyCurrentLimit(Pivot.kMotorSupplyCurrent)
                      .withStatorCurrentLimitEnable(Pivot.kMotorEnableCurrentLimit)
                      .withSupplyCurrentLimitEnable(Pivot.kMotorEnableCurrentLimit))
                      
    .withVoltage(new VoltageConfigs()
                    .withPeakForwardVoltage(12.0)
                    .withPeakReverseVoltage(-12.0)) 
    //--------------Motor Configs-------------------
    .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(Pivot.kMotorNeutralMode))
    //--------------Closed Loop General--------------
    .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
     
    .withContinuousWrap(false))
      
    //--------------Feedback Sensor-----------------
    .withFeedback(new FeedbackConfigs()
                  .withSensorToMechanismRatio(Pivot.kMotorGearRatio))

    .withSoftwareLimitSwitch(
      new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(2.22)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0.0))
    
    //--------------Closed Loop Ramp-------------
    .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(Pivot.kClosedLoopRamp))

    //-----------PID Slot 0------------------
    .withSlot0(new Slot0Configs()
                .withKS(Pivot.kSlot0kS)
                .withKV(Pivot.kSlot0kV)
                .withKP(Pivot.kSlot0kP)
                .withKI(Pivot.kSlot0kI)
                .withKD(Pivot.kSlot0kD))
  
    //-----------PID Slot 1------------------
    .withSlot1(new Slot1Configs()
                .withKS(Pivot.kSlot1kS)
                .withKV(Pivot.kSlot1kV)
                .withKP(Pivot.kSlot1kP)
                .withKI(Pivot.kSlot1kI)
                .withKD(Pivot.kSlot1kD))
    
    //---------------Motion Control------------
    .withMotionMagic(new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(RotationsPerSecond.of(20.0))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10.0))
                    .withMotionMagicJerk((RotationsPerSecondPerSecond.per(Seconds)).of(100)));
    
    /* Apply configuration */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = pivotMotor.getConfigurator().apply(toConfigure);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply articulator configs, error code: " + status.toString());
      DriverStation.reportWarning("Could not apply articulator configs, error code: " + status.toString(), false);
    }
  }

}
