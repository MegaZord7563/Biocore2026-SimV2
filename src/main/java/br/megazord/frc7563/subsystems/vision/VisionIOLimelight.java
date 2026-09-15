// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.vision;

import br.megazord.frc7563.Constants.VisionConstants.CameraConfig;
import br.megazord.frc7563.util.LimelightHelpers;
import br.megazord.frc7563.util.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

/**
 * Real {@link VisionIO} implementation for a Limelight, built on the vendored {@code
 * LimelightHelpers}.
 *
 * <p>
 * Reads three independent things every loop:
 * <ul>
 * <li><b>Global pose (MegaTag2)</b> via {@code getBotPoseEstimate_*_MegaTag2}. Requires the
 * robot's current heading to have been pushed via {@link #setRobotOrientation} first - without
 * that call, MT2 has no gyro reference and the pose will be stale/zeroed. This is the
 * continuous, high-accuracy source once the robot is already tracking well.</li>
 * <li><b>Seed pose (MegaTag1)</b> via {@code getBotPoseEstimate_*}, the non-orientation-fed
 * variant. Noisier and prone to rotational ambiguity with few tags, but needs no gyro input at
 * all - exists purely so {@code VisionSubsystem} can bootstrap the pose estimator right after
 * code start, before gyro yaw is trustworthy enough to feed MT2.</li>
 * <li><b>Local target</b> straight from {@code targetpose_robotspace} / tx / ty / ta, also
 * gyro-independent. Stays available even if orientation feeding stops or the estimator is being
 * reset.</li>
 * </ul>
 */
public class VisionIOLimelight implements VisionIO {

  private final String name;

  /** Whichever alliance-origin botpose to read for both global (MT2) and seed (MT1) position. */
  public enum AllianceOrigin {
    BLUE,
    RED
  }

  private final AllianceOrigin allianceOrigin;

  private static final double kHeartbeatTimeoutSeconds = 1.0;
  private double lastHeartbeatValue = -1.0;
  private double lastHeartbeatChangeTimestamp = 0.0;

  /**
   * @param config         this camera's name, mount offset and IMU mode - see {@code
   *                       VisionConstants.kCameras}. Mount offset and IMU mode are pushed to the
   *                       camera once, here in the constructor.
   * @param allianceOrigin which alliance-origin botpose to read. Almost always {@code BLUE} -
   *                       WPILib/PathPlanner standardize on the blue origin regardless of
   *                       alliance color, see {@code VisionConstants}.
   */
  public VisionIOLimelight(CameraConfig config, AllianceOrigin allianceOrigin) {
    this.name = config.name();
    this.allianceOrigin = allianceOrigin;
    configureCameraPose(config.robotToCamera());
    setImuMode(config.imuMode());
  }

  @Override
  public void configureCameraPose(Transform3d robotToCamera) {
    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        robotToCamera.getX(),
        robotToCamera.getY(),
        robotToCamera.getZ(),
        Units.radiansToDegrees(robotToCamera.getRotation().getX()),
        Units.radiansToDegrees(robotToCamera.getRotation().getY()),
        Units.radiansToDegrees(robotToCamera.getRotation().getZ()));
  }

  @Override
  public void setImuMode(int mode) {
    LimelightHelpers.SetIMUMode(name, mode);
  }

  @Override
  public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
    LimelightHelpers.SetRobotOrientation(name, yawDegrees, yawRateDegPerSec, 0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    updateConnected(inputs);
    updateGlobalPose(inputs);
    updateSeedPose(inputs);
    updateLocalTarget(inputs);
  }

  private void updateConnected(VisionIOInputs inputs) {
    double heartbeat = LimelightHelpers.getHeartbeat(name);
    double now = Timer.getFPGATimestamp();

    if (heartbeat != lastHeartbeatValue) {
      lastHeartbeatValue = heartbeat;
      lastHeartbeatChangeTimestamp = now;
    }

    inputs.connected = (now - lastHeartbeatChangeTimestamp) < kHeartbeatTimeoutSeconds;
  }

  private void updateGlobalPose(VisionIOInputs inputs) {
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    inputs.hasGlobalPose = estimate != null && estimate.tagCount > 0;
    if (!inputs.hasGlobalPose) {
      return;
    }

    inputs.globalPose = estimate.pose;
    inputs.globalPoseTimestampSeconds = estimate.timestampSeconds;
    inputs.globalTagCount = estimate.tagCount;
    inputs.globalAvgTagDistanceMeters = estimate.avgTagDist;
    inputs.globalAvgTagAreaPercent = estimate.avgTagArea;
  }

  private void updateSeedPose(VisionIOInputs inputs) {
    // Deliberately the non-MegaTag2 estimate: no SetRobotOrientation call needed, so this stays
    // valid even before the gyro has a trustworthy yaw (e.g. the instant code starts).
    PoseEstimate estimate = allianceOrigin == AllianceOrigin.BLUE
        ? LimelightHelpers.getBotPoseEstimate_wpiBlue(name)
        : LimelightHelpers.getBotPoseEstimate_wpiRed(name);

    inputs.hasSeedPose = estimate != null && estimate.tagCount > 0;
    if (!inputs.hasSeedPose) {
      return;
    }

    inputs.seedPose = estimate.pose;
    inputs.seedPoseTimestampSeconds = estimate.timestampSeconds;
    inputs.seedTagCount = estimate.tagCount;
  }

  private void updateLocalTarget(VisionIOInputs inputs) {
    inputs.hasTarget = LimelightHelpers.getTV(name);
    if (!inputs.hasTarget) {
      inputs.targetTagId = -1;
      return;
    }

    inputs.targetTagId = (int) LimelightHelpers.getFiducialID(name);
    inputs.targetInRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(name);
    inputs.targetHorizontalOffsetDeg = LimelightHelpers.getTX(name);
    inputs.targetVerticalOffsetDeg = LimelightHelpers.getTY(name);
    inputs.targetAreaPercent = LimelightHelpers.getTA(name);
  }
}
