// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.feeder;

import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * All constants for the feeder (leader + follower rollers, aligned to spin
 * the same direction). Split out of the monolithic {@code Constants} class
 * so the feeder's numbers live next to the feeder's code, the same way
 * {@code IntakeConstants} and {@code ShooterConstants} already do for their
 * subsystems.
 *
 * <p>Values were carried over from the old
 * {@code Constants.SubsystemsConstants.FeederConstants} block.
 */
public final class FeederConstants {

  private FeederConstants() {
  }

  /* motors - leader drives, follower mirrors it (motors are mechanically aligned) */
  public static final int kLeaderMotorId = 15;
  public static final int kFollowerMotorId = 22;
  public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

  /* current limit */
  public static final double kMotorThresholdCurrent = 120;
  public static final double kMotorSupplyCurrent = 70;
  public static final boolean kMotorEnableCurrentLimit = true;

  /* feedback sensor */
  public static final double kMotorGearRatio = 2.0;
  public static final boolean kContinuousWrap = false;

  /* PID slot 0 - main velocity control */
  public static final double kSlot0kS = 0.41674;
  public static final double kSlot0kV = 0.04235;
  public static final double kSlot0kP = 0.065389;
  public static final double kSlot0kI = 0.0;
  public static final double kSlot0kD = 0.0;

  /* speeds */
  public static final double kFeederRunRPS = 100;
  public static final double kFeederRunRadsPerSec = kFeederRunRPS * 2 * Math.PI;
}
