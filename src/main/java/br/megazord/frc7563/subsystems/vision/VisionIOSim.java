// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.vision;

/**
 * Sim/no-op {@link VisionIO}. Reports no target and no global pose - it exists so {@code
 * RobotContainer}'s {@code SIM} branch can construct a {@code VisionSubsystem} without a real
 * Limelight, the same role {@code SwerveModuleIOSim} plays for the drivetrain. It does not
 * simulate AprilTag detections; if simulated vision is ever needed (testing auto-align logic
 * without hardware, for example), replace this with a PhotonVision-sim-backed IO instead of
 * teaching this class to fake tag geometry.
 */
public class VisionIOSim implements VisionIO {
  // All methods use VisionIO's no-op defaults - connected/hasTarget/hasGlobalPose stay false.
}
