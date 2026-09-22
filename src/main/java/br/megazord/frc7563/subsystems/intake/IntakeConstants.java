// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

/**
 * All constants for the intake (pivot + rollers). Split out of the
 * monolithic {@code Constants} class so the intake's numbers live next to
 * the intake's code, the same way {@code ShooterConstants} already
 * separates Turret/Hood/Flywheel into their own file.
 *
 * <p>Values were carried over from the old {@code Constants.SubsystemsConstants.IntakeConstants}
 * block (where the pivot was called "Articulator" and the rollers were
 * called "Intake"). They still need to be re-verified/re-tuned against the
 * new pivot/rollers naming and gearbox before trusting them on the real
 * robot.
 *
 * <p>Layout:
 * <ul>
 *   <li>{@link Pivot} - the arm that deploys/stows the intake</li>
 *   <li>{@link Rollers} - the rollers that grab/eject the game piece</li>
 * </ul>
 */
public final class IntakeConstants {

  private IntakeConstants() {
  }

  /** The arm that deploys/stows the intake. */
  public static final class Pivot {
    private Pivot() {
    }

    /* motor */
    public static final int kMotorId = 20;
    public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

    /* current limit */
    public static final double kMotorThresholdCurrent = 80;
    public static final double kMotorSupplyCurrent = 30;
    public static final boolean kMotorEnableCurrentLimit = true;

    /* feedback sensor */
    public static final double kMotorGearRatio = 8.57; // new gearbox ratio 04/25/2026 (9 on gearbox + 21/20 pulley)
    public static final double kClosedLoopRamp = 0.25;

    /* PID slot 0 - normal move */
    public static final double kSlot0kS = 1.4756;
    public static final double kSlot0kV = 0.0;
    public static final double kSlot0kP = 30;
    public static final double kSlot0kI = 0.0;
    public static final double kSlot0kD = 5.0526;

    /* PID slot 1 - alternate gains (e.g. characterization), not used yet */
    public static final double kSlot1kS = 1.4756;
    public static final double kSlot1kV = 0.0;
    public static final double kSlot1kP = 160;
    public static final double kSlot1kI = 0.0;
    public static final double kSlot1kD = 5.0526;

    /* soft limits */
    public static final boolean kForwardSoftLimitEnable = true;
    public static final boolean kReverseSoftLimitEnable = true;
    public static final double kForwardSoftLimitThreshold = Units.degreesToRotations(180);
    public static final double kReverseSoftLimitThreshold = 0;

    /* positions (rad), ex-articulatorUpAngle/DownAngle/StartAngle */
    public static final double kStowedPositionRads = 0.04  * 2 * Math.PI;
    public static final double kDeployedPositionRads = 2.05 * 2 * Math.PI;
    public static final double kStartPositionRads = 2.15 *  2 * Math.PI;
  }

  /** The rollers that grab/eject the game piece. */
  public static final class Rollers {
    private Rollers() {
    }

    /* motor */
    public static final int kMotorId = 21;
    public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

    /* current limit */
    public static final double kMotorThresholdCurrent = 120;
    public static final double kMotorSupplyCurrent = 70;
    public static final boolean kMotorEnableCurrentLimit = true;

    /* feedback sensor */
    public static final double kMotorGearRatio = 1;
    public static final double kClosedLoopRamp = 0;

    /* PID */
    public static final double kS = 0.41978;
    public static final double kV = 0.043145;
    public static final double kP = 0.068582;
    public static final double kI = 0;
    public static final double kD = 0;

    /* speeds (rad/s), carried over verbatim from the old IntakeModeSpeed/OutakeModeSpeed - unit not re-verified */
    public static final double kIntakeSpeedRadsPerSec = 100;
    public static final double kOuttakeSpeedRadsPerSec = -50;
  }
}
