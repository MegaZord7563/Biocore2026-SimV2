// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * One physical camera: owns its IO and logged inputs, and exposes both the global
 * (field-relative) and local (robot-relative) readings described in {@link VisionIO}'s class
 * javadoc. Mirrors the role {@code SwerveModule} plays for a single swerve module -
 * {@link VisionSubsystem} owns one or more of these the same way {@code SwerveSubsystem} owns
 * four {@code SwerveModule}s.
 */
public class VisionCamera {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private final String name;

  private final Alert disconnectedAlert;

  public VisionCamera(VisionIO io, String name) {
    this.io = io;
    this.name = name;
    disconnectedAlert = new Alert("Disconnected vision camera: " + name, AlertType.kError);
  }

  void periodic(double yawDegrees, double yawRateDegPerSec) {
    io.setRobotOrientation(yawDegrees, yawRateDegPerSec);
    io.updateInputs(inputs);
    Logger.processInputs("Vision/" + name, inputs);

    disconnectedAlert.set(!inputs.connected && RobotConstants.enableAlerts);
  }

  public String getName() {
    return name;
  }

  // ---------------------------------------------------------------------
  // Global (field-relative) position
  // ---------------------------------------------------------------------

  public boolean hasGlobalPose() {
    return inputs.connected && inputs.hasGlobalPose;
  }

  public Pose2d getGlobalPose() {
    return inputs.globalPose;
  }

  public double getGlobalPoseTimestampSeconds() {
    return inputs.globalPoseTimestampSeconds;
  }

  public int getGlobalTagCount() {
    return inputs.globalTagCount;
  }

  public double getGlobalAvgTagDistanceMeters() {
    return inputs.globalAvgTagDistanceMeters;
  }

  public double getGlobalAvgTagAreaPercent() {
    return inputs.globalAvgTagAreaPercent;
  }

  // ---------------------------------------------------------------------
  // Seed position - see VisionIO's class javadoc
  // ---------------------------------------------------------------------

  public boolean hasSeedPose() {
    return inputs.connected && inputs.hasSeedPose;
  }

  public Pose2d getSeedPose() {
    return inputs.seedPose;
  }

  public double getSeedPoseTimestampSeconds() {
    return inputs.seedPoseTimestampSeconds;
  }

  public int getSeedTagCount() {
    return inputs.seedTagCount;
  }

  // ---------------------------------------------------------------------
  // Local (robot-relative) target tracking
  // ---------------------------------------------------------------------

  public boolean hasTarget() {
    return inputs.connected && inputs.hasTarget;
  }

  public int getTargetTagId() {
    return inputs.targetTagId;
  }

  /** Primary in-view tag's pose expressed in the robot's own frame (x fwd, y left, z up). */
  public Pose3d getTargetInRobotSpace() {
    return inputs.targetInRobotSpace;
  }

  public double getTargetHorizontalOffsetDeg() {
    return inputs.targetHorizontalOffsetDeg;
  }

  public double getTargetVerticalOffsetDeg() {
    return inputs.targetVerticalOffsetDeg;
  }

  public double getTargetAreaPercent() {
    return inputs.targetAreaPercent;
  }
}
