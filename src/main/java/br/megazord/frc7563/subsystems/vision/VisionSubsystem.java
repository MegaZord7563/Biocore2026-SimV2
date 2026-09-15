// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.vision;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;

import br.megazord.frc7563.Constants.VisionConstants;
import br.megazord.frc7563.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Owns every vision camera on the robot. Singleton, matching the pattern used by
 * {@link RobotState} in this codebase.
 *
 * <p>
 * Each loop, every camera reports its own candidate <b>global</b> (field-relative) pose; only
 * the single best candidate - by largest average tag area, among candidates that clear {@link
 * #selectBestGlobalPose}'s filter - gets fed into {@link RobotState}'s pose estimator alongside
 * odometry. This "best of N" selection is carried over as-is from the old setup's
 * {@code addPoseVisionNew()}: rather than fusing every trustworthy camera's reading
 * independently, only the one measurement most likely to be good gets used per loop.
 *
 * <p>
 * Right after code start, before the gyro has a trustworthy yaw to feed MT2, {@link
 * #trySeedPose} uses a few gyro-independent MT1 reads to snap the estimator to roughly the
 * right pose instead of starting at (0,0,0) and drifting into correctness.
 *
 * <p>
 * <b>Local</b> (robot-relative) target tracking is exposed directly via {@link #getCameraWithTarget}
 * and never touches {@code RobotState} - a command that wants to align on whatever tag is
 * currently in view reads it straight from here, so it keeps working even if global pose is
 * being rejected this loop or the estimator is mid-reset. See {@link VisionIO}'s class javadoc
 * for the full reasoning behind keeping these two paths separate.
 */
public class VisionSubsystem extends SubsystemBase {
  private static VisionSubsystem instance;

  private final List<VisionCamera> cameras = new ArrayList<>();
  private Supplier<Rotation2d> headingSupplier;
  private Supplier<Rotation2d> angularVelocitySupplier;
  private boolean initialized = false;

  private final RobotState robotState = RobotState.getInstance();
  private int seedReadingsRemaining = VisionConstants.kSeedReadingCount;

  private VisionSubsystem() {
  }

  public static VisionSubsystem getInstance() {
    if (instance == null) {
      instance = new VisionSubsystem();
    }
    return instance;
  }

  /**
   * Initializes this subsystem with its heading suppliers and cameras. Must be called once,
   * typically from {@code RobotContainer}, before {@link #periodic} does anything useful -
   * mirrors {@code RobotState.initializePoseEstimator}'s lifecycle. Safe to call more than once
   * only if you intend to fully replace the camera set; otherwise a no-op after the first call.
   *
   * @param headingSupplier         current robot heading, field-relative - e.g.
   *                                {@code swerveDrive::getGyroAngle}. Pushed to every camera
   *                                each loop so MegaTag2 can use gyro yaw instead of solving
   *                                rotation from tag geometry alone.
   * @param angularVelocitySupplier current robot angular velocity, wrapped in a
   *                                {@code Rotation2d} the same way
   *                                {@code SwerveSubsystem.getAngularVelocity()} does - e.g.
   *                                {@code swerveDrive::getAngularVelocity}.
   * @param cameras                 every camera on the robot. Empty is fine.
   */
  public void initialize(Supplier<Rotation2d> headingSupplier, Supplier<Rotation2d> angularVelocitySupplier,
      List<VisionCamera> cameras) {
    if (initialized) {
      return;
    }

    this.headingSupplier = headingSupplier;
    this.angularVelocitySupplier = angularVelocitySupplier;
    this.cameras.addAll(cameras);
    initialized = true;
  }

  @Override
  public void periodic() {
    if (!initialized) {
      return;
    }

    double yawDegrees = headingSupplier.get().getDegrees();
    double yawRateDegPerSec = angularVelocitySupplier.get().getDegrees();

    // Update every camera's inputs (local target tracking included) regardless of what happens
    // below - alignment commands need that data even on loops where global pose is rejected.
    for (VisionCamera camera : cameras) {
      camera.periodic(yawDegrees, yawRateDegPerSec);
    }

    try {
      // While spinning fast, MT2's yaw-derived solve is least reliable (motion blur, gyro lag) -
      // skip global pose entirely for the loop. Matches the old rejection threshold.
      boolean doRejectUpdate = Math.abs(yawRateDegPerSec) > VisionConstants.kMaxYawRateForVisionDegPerSec;

      if (!doRejectUpdate) {
        VisionCamera best = selectBestGlobalPose();
        if (best != null) {
          robotState.addVisionObservation(
              best.getGlobalPose(),
              best.getGlobalPoseTimestampSeconds(),
              stdDevsFor(best));
        }
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to process vision cameras", e.getStackTrace());
    }

    trySeedPose();
  }

  /**
   * Picks the single best global pose candidate across every camera this loop: among cameras
   * reporting at least one tag closer than {@code kBestPoseMaxTagDistanceMeters}, the one with
   * the largest average tag area wins (bigger tag in frame = more pixels = more precise solve).
   * Everyone else's reading is discarded for the loop, not fused. Returns {@code null} if no
   * camera has a valid candidate.
   */
  private VisionCamera selectBestGlobalPose() {
    return cameras.stream()
        .filter(VisionCamera::hasGlobalPose)
        .filter(c -> c.getGlobalTagCount() > 0
            && c.getGlobalAvgTagDistanceMeters() < VisionConstants.kBestPoseMaxTagDistanceMeters)
        .max(Comparator.comparingDouble(VisionCamera::getGlobalAvgTagAreaPercent))
        .orElse(null);
  }

  /**
   * For the first {@code VisionConstants.kSeedReadingCount} good MT1 (gyro-independent) reads
   * after code start, snaps the pose estimator hard to vision instead of gently blending it in -
   * this is what lets the robot boot up already knowing roughly where it is, instead of starting
   * at (0,0,0) and drifting into correctness over several seconds. Stops permanently once used
   * up, the same way the old setup's boot-time seeding window closed after ~1s.
   */
  private void trySeedPose() {
    if (seedReadingsRemaining <= 0) {
      return;
    }

    for (VisionCamera camera : cameras) {
      if (camera.hasSeedPose() && camera.getSeedTagCount() >= VisionConstants.kSeedTagCountRequired) {
        robotState.addVisionObservation(
            camera.getSeedPose(),
            camera.getSeedPoseTimestampSeconds(),
            VecBuilder.fill(VisionConstants.kSeedXyStdDev, VisionConstants.kSeedXyStdDev,
                VisionConstants.kSeedThetaStdDev));
        seedReadingsRemaining--;
        return;
      }
    }
  }

  /**
   * Scales xy trust in the selected global pose by distance and tag count: a distant tag gets a
   * wide (barely trusted) standard deviation, several close tags get a tight one. Theta is
   * deliberately NOT scaled the same way - see {@link VisionConstants#kGlobalThetaStdDev}'s
   * javadoc for why MT2 heading stays untrusted here regardless of tag count/distance.
   */
  private Matrix<N3, N1> stdDevsFor(VisionCamera camera) {
    double distance = camera.getGlobalAvgTagDistanceMeters();
    int tagCount = Math.max(camera.getGlobalTagCount(), 1);

    double xyStdDev = VisionConstants.kGlobalXyStdDevBase * (distance * distance) / tagCount;
    xyStdDev = Math.min(Math.max(xyStdDev, VisionConstants.kGlobalXyStdDevMin), VisionConstants.kGlobalXyStdDevMax);

    return VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.kGlobalThetaStdDev);
  }

  // ---------------------------------------------------------------------
  // Local (robot-relative) target tracking - independent of RobotState
  // ---------------------------------------------------------------------

  /** @return the first camera with a target currently in view, or {@code null} if none do. */
  public VisionCamera getCameraWithTarget() {
    for (VisionCamera camera : cameras) {
      if (camera.hasTarget()) {
        return camera;
      }
    }
    return null;
  }

  public List<VisionCamera> getCameras() {
    return cameras;
  }
}
