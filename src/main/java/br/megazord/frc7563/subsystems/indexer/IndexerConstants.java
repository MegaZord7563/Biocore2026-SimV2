// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.indexer;

import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Constants for the indexer subsystem.
 * Split out into its own subsystem folder following the team's modular architecture.
 */
public final class IndexerConstants {

  private IndexerConstants() {
  }

  /* motor info */
  public static final int motorIndexerPort = 14;
  public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Brake;

  /* current limit */
  public static final double kMotorThresholdCurrent = 60.0;
  public static final boolean kMotorEnableCurrentLimit = true;

  /* feedback sensor */
  public static final double kMotorGearRatio = 1.0;
  public static final boolean kContinuousWrap = false;

  /* PID slot 0 - velocity control */
  public static final double kSlot0kS = 0.47179;
  public static final double kSlot0kV = 0.043922;
  public static final double kSlot0kP = 0.10941;
  public static final double kSlot0kI = 0.0;
  public static final double kSlot0kD = 0.0;

  /* speeds */
  public static final double kIndexerRunRPS = 50.0;
  public static final double kIndexerRunRadsPerSec = kIndexerRunRPS * 2 * Math.PI;
}
