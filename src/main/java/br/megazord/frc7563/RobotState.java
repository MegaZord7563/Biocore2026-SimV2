// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * RobotState is the single source of truth for the robot's state: odometry /
 * pose estimation today, and a home for other cross-subsystem state
 * (measured chassis speeds, mechanism flags, etc.) as it's needed.
 *
 * Subsystems (SwerveSubsystem, vision, mechanisms...) feed observations into
 * RobotState; anything that needs to *read* robot state (commands,
 * PathPlanner, dashboards) reads from here instead of poking at a
 * subsystem's internals.
 *
 * This is a singleton, matching the existing pattern used by
 * SwerveSubsystem in this codebase.
 */
public class RobotState {
  private static RobotState instance;

  // ---------------------------------------------------------------------
  // Pose / Odometry
  // ---------------------------------------------------------------------

  /**
   * Standard deviations for the odometry and vision measurements.
   */
  private static final Matrix<N3, N1> odometryStdDevs = VecBuilder.fill(0.015, 0.015, (10 * Math.PI) / 180);
  private static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.15, 0.15, (5 * Math.PI) / 180);

  private SwerveDrivePoseEstimator poseEstimator;
  private boolean poseEstimatorInitialized = false;

  // ---------------------------------------------------------------------
  // Chassis speeds
  // ---------------------------------------------------------------------

  private ChassisSpeeds measuredChassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeChassisSpeeds = new ChassisSpeeds();

  // ---------------------------------------------------------------------
  // Mechanism / misc state
  // ---------------------------------------------------------------------
  // Add fields + getters/setters here as mechanisms come online this season,
  // e.g.:
  // private boolean hasGamePiece = false;
  // public boolean hasGamePiece() { return hasGamePiece; }
  // public void setHasGamePiece(boolean hasGamePiece) { this.hasGamePiece = hasGamePiece; }

  private RobotState() {
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  // ---------------------------------------------------------------------
  // Pose estimator lifecycle
  // ---------------------------------------------------------------------

  /**
   * Initializes the pose estimator. Must be called once, typically from
   * SwerveSubsystem's constructor, before {@link #addOdometryObservation} is
   * used. Safe to call more than once only if you intend to fully re-create
   * the estimator (e.g. re-configuring drive hardware); otherwise it's a
   * no-op after the first call.
   *
   * @param kinematics      Swerve drive kinematics.
   * @param gyroAngle       Current gyro angle.
   * @param modulePositions Current module positions.
   * @param initialPose     Pose to start at (always towards the blue wall in
   *                        this codebase; the field origin never flips).
   */
  public void initializePoseEstimator(SwerveDriveKinematics kinematics, Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions, Pose2d initialPose) {
    if (poseEstimatorInitialized) {
      return;
    }

    poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyroAngle,
        modulePositions,
        initialPose,
        odometryStdDevs,
        visionStdDevs);

    poseEstimatorInitialized = true;
  }

  /**
   * Feeds a new odometry sample into the pose estimator. Call this once per
   * periodic loop from SwerveSubsystem.
   *
   * @param gyroAngle       Current gyro angle.
   * @param modulePositions Current module positions.
   */
  public void addOdometryObservation(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    if (!poseEstimatorInitialized) {
      return;
    }
    poseEstimator.update(gyroAngle, modulePositions);
  }

  /**
   * Feeds a vision-based pose measurement into the pose estimator, using the
   * default vision standard deviations.
   *
   * @param visionPose       Pose measured by the vision system, field
   *                         relative.
   * @param timestampSeconds FPGA timestamp of the measurement.
   */
  public void addVisionObservation(Pose2d visionPose, double timestampSeconds) {
    addVisionObservation(visionPose, timestampSeconds, visionStdDevs);
  }

  /**
   * Feeds a vision-based pose measurement into the pose estimator, with
   * custom standard deviations (e.g. scaled by distance to tag).
   *
   * @param visionPose       Pose measured by the vision system, field
   *                         relative.
   * @param timestampSeconds FPGA timestamp of the measurement.
   * @param stdDevs          Standard deviations to trust this measurement
   *                         with.
   */
  public void addVisionObservation(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
    if (!poseEstimatorInitialized) {
      return;
    }
    poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
  }

  /**
   * Resets the pose estimator to a new location.
   *
   * @param gyroAngle       Current gyro angle.
   * @param modulePositions Current module positions.
   * @param pose            Pose to reset the odometry to.
   */
  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    if (!poseEstimatorInitialized) {
      return;
    }
    poseEstimator.resetPosition(gyroAngle, modulePositions, pose);
  }

  /**
   * @return the current estimated field-relative pose.
   */
  @AutoLogOutput(key = "RobotState/PoseEstimator")
  public Pose2d getEstimatedPose() {
    return poseEstimatorInitialized ? poseEstimator.getEstimatedPosition() : new Pose2d();
  }

  // ---------------------------------------------------------------------
  // Chassis speeds
  // ---------------------------------------------------------------------

  /**
   * Updates the cached robot-relative chassis speeds. Call once per periodic
   * loop from SwerveSubsystem.
   */
  public void setMeasuredChassisSpeeds(ChassisSpeeds robotRelativeSpeeds) {
    this.measuredChassisSpeeds = robotRelativeSpeeds;
    this.fieldRelativeChassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getEstimatedPose().getRotation());
  }

  @AutoLogOutput(key = "RobotState/ChassisSpeeds/Measured")
  public ChassisSpeeds getMeasuredChassisSpeeds() {
    return measuredChassisSpeeds;
  }

  @AutoLogOutput(key = "RobotState/ChassisSpeeds/FieldRelative")
  public ChassisSpeeds getFieldRelativeChassisSpeeds() {
    return fieldRelativeChassisSpeeds;
  }

  public Translation2d getSpeedTranslation() {
    return new Translation2d(fieldRelativeChassisSpeeds.vxMetersPerSecond, fieldRelativeChassisSpeeds.vyMetersPerSecond);
  }

  public Rotation2d getAngularVelocity() {
    return new Rotation2d(measuredChassisSpeeds.omegaRadiansPerSecond);
  }
}