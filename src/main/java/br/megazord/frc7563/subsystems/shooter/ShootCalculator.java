// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import br.megazord.frc7563.RobotState;
import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Calculator;
import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Field;
import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Geometry;

/**
 * Computes the turret angle, hood angle and flywheel speed needed to hit the
 * hub while the robot is moving, compensating for shot time-of-flight.
 *
 * This is a singleton, matching the existing pattern used by
 * {@link RobotState} in this codebase: there is exactly one shot solution
 * for the robot at any given time, so there is no reason for more than one
 * instance to exist.
 *
 * Fixes applied vs. the original ShootCalculator3:
 *  - The per-alliance target flipping was correct in spirit but called
 *    RobotContainer.swerveDrive, a private instance field, as if it were
 *    static (didn't compile) and depended on a FieldConstants class that
 *    doesn't exist in this codebase. It's rebuilt here using
 *    {@code com.pathplanner.lib.util.FlippingUtil} (already a project
 *    dependency, and what this codebase's own AutoBuilder mirroring uses)
 *    to flip the blue-alliance target to red, on demand, instead of
 *    maintaining a separately-tracked mutable target field.
 *  - The "not hub" case now genuinely picks the nearest of several corner
 *    targets (Field.kCornerTargets, alliance-flipped) via
 *    Translation2d#nearest, instead of only compiling against a manually
 *    assembled list that depended on the missing FieldConstants.
 *  - The look-ahead loop subtracted the predicted robot displacement/turn
 *    instead of adding it, contradicting its own "NO FUTURO = plus" comment
 *    (i.e. it was predicting the robot's *past* pose, not its future one).
 *    It now adds the displacement, so the turret actually leads the shot in
 *    the direction the robot is moving.
 *  - minDistance/maxDistance were declared but never used; the computed
 *    distance is now clamped to that range (Calculator.kMinDistanceMeters /
 *    kMaxDistanceMeters) before every map lookup so the interpolating maps
 *    are never queried outside their intended envelope.
 *  - getTragetTranslation() -> getTargetTranslation() (typo).
 *  - State (turret angle / hood angle / flywheel speed) is now instance
 *    state on the singleton instead of static fields on a class that could
 *    otherwise be freely instantiated.
 */
public class ShootCalculator {

  private static ShootCalculator instance;

  private final RobotState robotState = RobotState.getInstance();

  private boolean isHub = true;

  private Rotation2d mTurretAngle = new Rotation2d();
  private double mFlyWheelSpeed = 0.0;
  private double mHoodAngle = 0.0;

  /** Organizes the shot output and allows automatic interpolation between two solutions. */
  public record ShootParameters(Rotation2d turretTargetPosition, double hoodPosition, double shootSpeedRps) {
    public static ShootParameters interpolate(ShootParameters start, ShootParameters end, double t) {
      return new ShootParameters(
          start.turretTargetPosition().interpolate(end.turretTargetPosition(), t),
          MathUtil.interpolate(start.hoodPosition(), end.hoodPosition(), t),
          MathUtil.interpolate(start.shootSpeedRps(), end.shootSpeedRps(), t));
    }
  }

  private ShootCalculator() {
  }

  public static ShootCalculator getInstance() {
    if (instance == null) {
      instance = new ShootCalculator();
    }
    return instance;
  }

  /**
   * Convenience overload that pulls the current pose and field-relative
   * chassis speeds straight from {@link RobotState}, so callers (commands,
   * RobotContainer) don't need to reach into the swerve subsystem directly.
   */
  public ShootParameters calculateMovingShot() {
    return calculateMovingShot(robotState.getEstimatedPose(), robotState.getFieldRelativeChassisSpeeds());
  }

  /**
   * Calculates the shot parameters compensating for robot motion (iteration +
   * interpolation).
   */
  public ShootParameters calculateMovingShot(Pose2d robotPose, ChassisSpeeds fieldSpeeds) {
    Translation2d target = getTargetTranslation(robotPose.getTranslation());

    // 1. Initial distance/time-of-flight estimate.
    double distance = clampToShotEnvelope(robotPose.getTranslation().getDistance(target));
    double timeOfFlight = Calculator.kTimeOfFlightMap.get(distance);

    Pose2d futurePose = robotPose;
    Pose2d turretPose = futurePose.transformBy(Geometry.kRobotToTurret2d);

    // 2. Iterate to converge on where the robot (and turret) will actually be
    // once the shot arrives.
    for (int i = 0; i < 8; i++) {

      Translation2d robotDisplacement = new Translation2d(
          fieldSpeeds.vxMetersPerSecond * timeOfFlight,
          fieldSpeeds.vyMetersPerSecond * timeOfFlight);

      // Future pose = current pose + how far the robot travels during the
      // shot's time of flight.
      futurePose = new Pose2d(
          robotPose.getTranslation().plus(robotDisplacement),
          robotPose.getRotation().plus(
              Rotation2d.fromRadians(fieldSpeeds.omegaRadiansPerSecond * (timeOfFlight + RobotConstants.loopPeriodSecs)))); // Tof + Latency

      // Turret offset applied in the same frame as the distance calculation.
      turretPose = futurePose.transformBy(Geometry.kRobotToTurret2d);

      distance = clampToShotEnvelope(target.getDistance(turretPose.getTranslation()));

      timeOfFlight = Calculator.kTimeOfFlightMap.get(distance);
    }

    SmartDashboard.putNumber("Shooter Calculator/Distance", distance);
    SmartDashboard.putNumber("Shooter Calculator/Time of Flight", timeOfFlight);

    Translation2d shooterToTarget = target.minus(turretPose.getTranslation());
    Rotation2d turretAngle = shooterToTarget.getAngle();

    double finalHood = isHub ? Calculator.kHoodAngleMap.get(distance) : 160.0;
    double finalSpeed = Calculator.kFlywheelSpeedMap.get(distance) - (isHub ? 0.0 : 10.0);

    mTurretAngle = turretAngle;
    mFlyWheelSpeed = finalSpeed;
    mHoodAngle = finalHood;

    return new ShootParameters(turretAngle, finalHood, finalSpeed);
  }

  private static double clampToShotEnvelope(double distanceMeters) {
    return MathUtil.clamp(
        distanceMeters,
        Calculator.kMinDistanceMeters,
        Calculator.kMaxDistanceMeters);
  }

  public Rotation2d getTurretAngle() {
    return mTurretAngle;
  }

  public double getHoodAngle() {
    return mHoodAngle;
  }

  public double getFlywheelSpeed() {
    return mFlyWheelSpeed;
  }

  /**
   * Current shot target: the hub when {@link #aimToHub} is true, otherwise
   * whichever corner target is closest to the robot right now. Both are
   * defined blue-side in {@link ShooterConstants.Field} and flipped to the
   * red side of the field here once the current alliance is known -- this
   * codebase's pose estimator itself never flips origin (see
   * RobotState#initializePoseEstimator), but the physical target does move
   * from one alliance's side of the field to the other's.
   */
  public Translation2d getTargetTranslation() {
    return getTargetTranslation(robotState.getEstimatedPose().getTranslation());
  }

  private Translation2d getTargetTranslation(Translation2d robotTranslation) {
    aimToHub(!robotState.isInsideNeutralShootingZone());
    boolean isRedAlliance = DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;

    if (isHub) {
      return isRedAlliance ? FlippingUtil.flipFieldPosition(Field.kHubCenter) : Field.kHubCenter;
    }

    return robotTranslation.nearest(allianceCornerTargets(isRedAlliance));
  }

  private static List<Translation2d> allianceCornerTargets(boolean isRedAlliance) {
    if (!isRedAlliance) {
      return Field.kCornerTargets;
    }

    List<Translation2d> flipped = new ArrayList<>(Field.kCornerTargets.size());
    for (Translation2d blueSideCorner : Field.kCornerTargets) {
      flipped.add(FlippingUtil.flipFieldPosition(blueSideCorner));
    }
    return flipped;
  }

  /**
   * @return isHub true to aim/shoot at the hub (uses the interpolated hood
   *              angle map and full flywheel speed), false to shoot at
   *              whichever corner target is currently closest to the robot,
   *              at the fixed, lower-speed shot used for that target.
   */
  public boolean aimToHub() {
    return isHub;
  }

  private boolean aimToHub(boolean isHub) {
    this.isHub = isHub;
    return isHub;
  }
}
