// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter;

import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/**
 * All constants for the shooter (turret + hood + flywheel + the moving-shot
 * calculator). Split out of the monolithic {@code Constants} class so the
 * shooter's numbers live next to the shooter's code, the same way this
 * package already separates Turret/Hood/Flywheel into their own files.
 *
 * <p>Layout:
 * <ul>
 *   <li>{@link Geometry} - where the turret sits on the robot</li>
 *   <li>{@link Field} - fixed field positions the shooter aims at</li>
 *   <li>{@link Calculator} - lookup tables/limits used by {@link ShootCalculator}</li>
 *   <li>{@link Turret}, {@link Hood}, {@link Flywheel} - per-mechanism motor/PID config</li>
 * </ul>
 */
public final class ShooterConstants {

  private ShooterConstants() {
  }

  /** Where the turret sits relative to the robot's center. */
  public static final class Geometry {
    private Geometry() {
    }

    public static final double kTurretOffsetXMeters = -0.1445;
    public static final double kTurretOffsetYMeters = -0.1516;
    public static final double kTurretOffsetZMeters = 0.364;

    public static final Transform3d kRobotToTurret3d = new Transform3d(
        new Translation3d(kTurretOffsetXMeters, kTurretOffsetYMeters, kTurretOffsetZMeters),
        Rotation3d.kZero);

    public static final Transform2d kRobotToTurret2d = new Transform2d(
        new Translation2d(kTurretOffsetXMeters, kTurretOffsetYMeters),
        new Rotation2d());
  }

  /**
   * Fixed field positions the shooter aims at, all given for the blue
   * alliance. ShootCalculator flips them to the red side (via
   * {@code com.pathplanner.lib.util.FlippingUtil}, keeping the same
   * blue-origin coordinate frame this codebase always uses -- see
   * RobotState#initializePoseEstimator and SwerveSubsystem's AutoBuilder
   * mirroring) once it knows the current alliance.
   */
  public static final class Field {
    private Field() {
    }

    /**
     * Hub position, blue alliance side.
     * competition; this is a placeholder so ShootCalculator compiles and runs.
     */
    public static final Translation2d kHubCenter = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));

    /**
     * Feed/corner positions to shoot into when not aiming at the hub, blue
     * alliance side. ShootCalculator targets whichever one is currently
     * closest to the robot.
     */
    public static final List<Translation2d> kCornerTargets = List.of(
        new Translation2d(1.0, 0.75),
        new Translation2d(1.0, 7.35));
  }

  /** Interpolated lookup tables and shot envelope used by {@link ShootCalculator}. */
  public static final class Calculator {
    private Calculator() {
    }

    public static final double kMinDistanceMeters = 1.34;
    public static final double kMaxDistanceMeters = 5.60;

    /** Distance (m) -> projectile time of flight (s). */
    public static final InterpolatingDoubleTreeMap kTimeOfFlightMap = new InterpolatingDoubleTreeMap();
    /** Distance (m) -> hood angle (deg). */
    public static final InterpolatingDoubleTreeMap kHoodAngleMap = new InterpolatingDoubleTreeMap();
    /** Distance (m) -> flywheel speed (rps). */
    public static final InterpolatingDoubleTreeMap kFlywheelSpeedMap = new InterpolatingDoubleTreeMap();

    static {
      kHoodAngleMap.put(1.0, 28.3);
      kHoodAngleMap.put(2.0, 28.3 * 2);
      kHoodAngleMap.put(3.0, 28.3 * 3);
      kHoodAngleMap.put(4.0, 28.3 * 4);
      kHoodAngleMap.put(5.0, 28.3 * 5);
      kHoodAngleMap.put(6.0, 160.0);

      kFlywheelSpeedMap.put(1.0, 48.2);
      kFlywheelSpeedMap.put(1.5, 51.5);
      kFlywheelSpeedMap.put(2.0, 53.6);
      kFlywheelSpeedMap.put(2.5, 56.8);
      kFlywheelSpeedMap.put(3.0, 58.2);
      kFlywheelSpeedMap.put(3.5, 62.5);
      kFlywheelSpeedMap.put(4.00, 66.00);
      kFlywheelSpeedMap.put(4.24, 63.66);
      kFlywheelSpeedMap.put(4.30, 63.10);
      kFlywheelSpeedMap.put(5.42, 71.38);
      kFlywheelSpeedMap.put(5.76, 71.67);

      kTimeOfFlightMap.put(1.0, 1.05);
      kTimeOfFlightMap.put(1.5, 1.00);
      kTimeOfFlightMap.put(2.0, 1.08);
      kTimeOfFlightMap.put(2.5, 1.09);
      kTimeOfFlightMap.put(3.0, 1.15);
      kTimeOfFlightMap.put(3.5, 1.08);
      kTimeOfFlightMap.put(4.0, 1.19);
      kTimeOfFlightMap.put(4.5, 1.25);
      kTimeOfFlightMap.put(5.0, 1.24);
      kTimeOfFlightMap.put(5.5, 1.26);
      kTimeOfFlightMap.put(6.0, 1.24);
    }
  }

  public static final class Turret {
    private Turret() {
    }

    /* motor */
    public static final int kMotorId = 16;
    public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

    /* current limit */
    public static final double kMotorThresholdCurrent = 60;
    public static final boolean kMotorEnableCurrentLimit = true;

    /* feedback sensor */
    public static final double kMotorGearRatio = 18.45; // 12.3 previously
    public static final boolean kContinuousWrap = false;

    /* PID */
    public static final double kClosedLoopRamp = 0.25;
    public static final double kS = 0.0; // 1.0069 previously
    public static final double kV = 0.0; // 0.0035301 previously
    public static final double kP = 80; // 65.619 previously
    public static final double kI = 0;
    public static final double kD = 0;

    /* soft limits */
    public static final boolean kForwardSoftLimitEnable = true;
    public static final boolean kReverseSoftLimitEnable = true;
    public static final double kForwardSoftLimitThreshold = Units.degreesToRotations(180);
    public static final double kReverseSoftLimitThreshold = -Units.degreesToRotations(180);
  }

  public static final class Hood {
    private Hood() {
    }

    /* motor */
    public static final int kMotorId = 18;
    public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;
    public static final MotorType kMotorType = MotorType.kBrushless;

    /* current limit */
    public static final double kMotorThresholdCurrent = 60;
    public static final boolean kMotorEnableCurrentLimit = true;

    /* feedback sensor */
    public static final double kMotorGearRatio = 2.833;
    public static final boolean kContinuousWrap = false;

    /* PID */
    public static final double kS = 0;
    public static final double kV = 0.00566;
    public static final double kP = 70;
    public static final double kI = 0;
    public static final double kD = 0;

    /* soft limits */
    public static final boolean kForwardSoftLimitEnable = true;
    public static final boolean kReverseSoftLimitEnable = false;
    public static final double kForwardSoftLimitThreshold = Units.degreesToRotations(160);
    public static final double kReverseSoftLimitThreshold = 0;
  }

  public static final class Flywheel {
    private Flywheel() {
    }

    /* motors - leader drives, follower mirrors it */
    public static final int kLeaderMotorId = 17;
    public static final int kFollowerMotorId = 19;
    public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

    /* current limit */
    public static final double kMotorThresholdCurrent = 120;
    public static final double kMotorSupplyCurrent = 70;
    public static final boolean kMotorEnableCurrentLimit = true;

    /* feedback sensor */
    public static final boolean kContinuousWrap = false;
    public static final double kMotorGearRatio = 1;

    /* PID slot 0 - main velocity control */
    public static final double kSlot0kS = 0.23121; // 0.32743 previously
    public static final double kSlot0kV = 0.12235; // 0.12664 previously
    public static final double kSlot0kP = 8.14487; // 0.14487 / 0.20002 previously
    public static final double kSlot0kI = 0;
    public static final double kSlot0kD = 0;

    /* PID slot 1 - alternate gains (e.g. characterization) */
    public static final double kSlot1kS = 0.23121;
    public static final double kSlot1kV = 0.12235;
    public static final double kSlot1kP = 0.14487;
    public static final double kSlot1kI = 0;
    public static final double kSlot1kD = 0;
  }
}
