// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** 
 * Interface for swerve module I/O operations.
 * 
 */
public interface IntakeIO 
{
  @AutoLog
  public static class IntakeIOInputs {
    public boolean pivotConnected = false;
    public double pivotPositionRads = 0.0;
    public double pivotVelocityRadsPerSec = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSupplyCurrentAmps = 0.0;
    public double pivotTempCelsius = 0.0;

    public boolean rollersConnected = false;
    public double rollerPositionRads = 0.0;
    public double rollersVelocityRadsPerSec = 0.0;
    public double rollersAppliedVolts = 0.0;
    public double rollersSupplyCurrentAmps = 0.0;
    public double rollersTempCelsius = 0.0;

  }

  public static enum IntakeIOOutputMode {
    RUN,
    MOVE,
    CHARACTERIZE,
    COAST,
    BREAK
  }

  public static class IntakeIOOutputs {
    public IntakeIOOutputMode mode = IntakeIOOutputMode.COAST;

    public double rollersSpeedRadPerSec = 0.0;
    public double pivotTargetPositionRads = 0.0;
    public double rollersCharacterizationOutput = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void applyOutputs(IntakeIOOutputs outputs) {}
}

