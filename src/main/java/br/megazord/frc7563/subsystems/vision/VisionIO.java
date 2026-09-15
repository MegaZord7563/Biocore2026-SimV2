// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * IO layer for a single vision camera, following the same {@code XyzIO} pattern used by
 * {@code GyroIO} and {@code SwerveModuleIO} in this codebase.
 *
 * <p>
 * Every camera can answer two different questions, and code that consumes this IO should be
 * deliberate about which one it's asking:
 *
 * <ul>
 * <li><b>Global position</b> - "where is the robot on the field?" A {@link Pose2d} in the
 * WPILib blue-origin field frame, meant to be fused into {@code RobotState}'s pose estimator
 * alongside odometry. This drifts-corrects the robot's estimated position over time but is
 * only as good as the estimator's trust in it (tag count, distance, ambiguity).</li>
 * <li><b>Local position</b> - "where is the target relative to me, right now?" A
 * {@link Pose3d}/{@code Transform3d}-shaped reading of the currently-seen tag in the robot's
 * own reference frame (or raw tx/ty/ta), independent of odometry or the pose estimator
 * entirely. Use this for direct alignment commands (e.g. drive-to-target) that need to work
 * even if the global pose estimate is temporarily untrustworthy.</li>
 * </ul>
 */
public interface VisionIO {

  @AutoLog
  class VisionIOInputs {
    /** Whether the camera is publishing fresh data (heartbeat check), not whether it sees a target. */
    boolean connected = false;

    // ---------------------------------------------------------------------
    // Global (field-relative) position - feeds RobotState's pose estimator
    // ---------------------------------------------------------------------

    /** True if this frame produced a usable field-relative pose (independent of hasTarget). */
    boolean hasGlobalPose = false;
    Pose2d globalPose = new Pose2d();
    /** FPGA timestamp the global pose corresponds to (capture time, latency already subtracted). */
    double globalPoseTimestampSeconds = 0.0;
    int globalTagCount = 0;
    double globalAvgTagDistanceMeters = 0.0;
    double globalAvgTagAreaPercent = 0.0;

    // ---------------------------------------------------------------------
    // Seed position - a gyro-INDEPENDENT global pose (MegaTag1), only trustworthy with several
    // tags in view. Exists purely to bootstrap the pose estimator right after code start, before
    // gyro yaw is trustworthy enough to feed MegaTag2. See VisionSubsystem.trySeedPose.
    // ---------------------------------------------------------------------

    boolean hasSeedPose = false;
    Pose2d seedPose = new Pose2d();
    double seedPoseTimestampSeconds = 0.0;
    int seedTagCount = 0;

    // ---------------------------------------------------------------------
    // Local (robot-relative) target tracking - independent of the pose estimator
    // ---------------------------------------------------------------------

    /** True if a target is currently in view (this frame's "tv"). */
    boolean hasTarget = false;
    int targetTagId = -1;
    /** Primary in-view tag's pose expressed in the robot's own frame (x fwd, y left, z up). */
    Pose3d targetInRobotSpace = new Pose3d();
    double targetHorizontalOffsetDeg = 0.0; // tx
    double targetVerticalOffsetDeg = 0.0; // ty
    double targetAreaPercent = 0.0; // ta
  }

  /** Reads the latest camera data into {@code inputs}. Call once per periodic loop. */
  default void updateInputs(VisionIOInputs inputs) {
  }

  /**
   * Feeds the camera the robot's current heading (and ideally yaw rate) so MegaTag2-style
   * global pose estimation can use gyro yaw instead of solving for rotation from the tag
   * geometry alone - this is what makes a single visible tag give a trustworthy global pose.
   * Call every periodic loop, before {@link #updateInputs}. No-op on IOs that don't need it.
   *
   * @param yawDegrees      current robot yaw in degrees, 0 = facing away from your alliance wall
   * @param yawRateDegPerSec current robot yaw rate in degrees/sec
   */
  default void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
  }

  /**
   * One-time setup: tells the camera where it's physically mounted, in robot-relative space.
   * Call once, right after construction - not every loop. No-op on IOs that don't need it (e.g.
   * IOs where the camera's mount pose is instead set once in that camera's own web UI).
   */
  default void configureCameraPose(Transform3d robotToCamera) {
  }

  /**
   * One-time setup: selects how the camera should combine its internal IMU (if it has one) with
   * the yaw pushed via {@link #setRobotOrientation}. Meaning is IO-specific - see {@code
   * VisionConstants.CameraConfig}'s javadoc for what the values mean for a Limelight. No-op on
   * IOs without a configurable IMU mode.
   */
  default void setImuMode(int mode) {
  }
}
