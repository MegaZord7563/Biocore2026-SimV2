// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.KrakenMotorConstants;
import br.megazord.frc7563.Constants.ModuleConstants;
import org.junit.jupiter.api.Test;

/**
 * Guards the drive gearing/gains constants referenced by {@link DriveConstants#driveGearRatio}'s
 * javadoc ("Guarded by DriveGainsTest").
 *
 * <p>
 * {@code driveGearRatio} exists solely so PathPlanner's {@code RobotConfig} sees the same
 * reduction the modules actually run. That only holds up if it keeps pointing at
 * {@link ModuleConstants#kDriveMotorGearRatio} rather than a hand-copied literal, and if the
 * wheel geometry PathPlanner uses stays consistent with the one {@link
 * br.megazord.frc7563.subsystems.swerve.SwerveConversions} uses. These tests also re-derive the
 * theoretical top speed from the raw gear ratio and motor free speed, which is the same chain
 * of numbers that silently scaled the whole drivetrain by 3.13x for a season (see {@code
 * SwerveConversions}'s javadoc) - a gear ratio typo here would reproduce that bug.
 */
class DriveGainsTest {

  private static final double DELTA = 1e-9;

  @Test
  void driveGearRatio_tracksModuleConstants() {
    // PathPlanner's RobotConfig must see the exact reduction the modules run. If this ever
    // becomes a separate literal instead of a reference, the two can silently drift apart.
    assertEquals(ModuleConstants.kDriveMotorGearRatio, DriveConstants.driveGearRatio, DELTA);
  }

  @Test
  void driveGearRatio_isInPlausibleSwerveRange() {
    // Sanity bound on the reduction itself: real swerve drive reductions sit roughly in the
    // 5:1-7:1 range for this class of module (MK4/MK4i/MK5n). A ratio outside a much wider
    // band is almost certainly a units/typo mistake, not a legitimate gearing choice.
    assertTrue(
        DriveConstants.driveGearRatio > 1.0 && DriveConstants.driveGearRatio < 15.0,
        "driveGearRatio (" + DriveConstants.driveGearRatio + ") is outside a plausible swerve "
            + "drive reduction range");
  }

  @Test
  void wheelGeometry_consistentBetweenPathPlannerAndSwerveConversions() {
    // PathPlanner's ModuleConfig is built from a wheel radius (kWheelDiameterMeters / 2), while
    // SwerveConversions works in circumference. They describe the same physical wheel and must
    // agree, or PathPlanner-driven autos and teleop odometry will disagree about distance.
    double radiusDerivedCircumference = (ModuleConstants.kWheelDiameterMeters / 2.0) * 2.0 * Math.PI;
    assertEquals(ModuleConstants.kWheelCircumferenceMeters, radiusDerivedCircumference, DELTA);
  }

  @Test
  void theoreticalTopSpeed_matchesManualDerivationFromGearRatioAndFreeSpeed() {
    // Re-derives kDriveWheelFreeSpeedMPS independently from the raw Kraken free speed, the gear
    // ratio and the wheel circumference. This is exactly the calculation chain that was broken
    // for a season (motor speed -> wheel speed -> ground speed), so a regression here is a
    // repeat of that bug.
    double motorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60.0;
    double wheelFreeSpeedRps = motorFreeSpeedRps / ModuleConstants.kDriveMotorGearRatio;
    double expectedTopSpeedMps = wheelFreeSpeedRps * ModuleConstants.kWheelCircumferenceMeters;

    assertEquals(expectedTopSpeedMps, ModuleConstants.kDriveWheelFreeSpeedMPS, DELTA);
  }

  @Test
  void physicalMaxSpeed_doesNotExceedTheoreticalFreeSpeed() {
    // The commanded max teleop speed can't physically exceed the motor's no-load top speed
    // for the configured gearing - if it does, either the gear ratio or the max speed constant
    // was changed without updating the other.
    assertTrue(
        DriveConstants.kPhysicalMaxSpeedMetersPerSecond <= ModuleConstants.kDriveWheelFreeSpeedMPS,
        "kPhysicalMaxSpeedMetersPerSecond (" + DriveConstants.kPhysicalMaxSpeedMetersPerSecond
            + ") exceeds the theoretical free-spin speed ("
            + ModuleConstants.kDriveWheelFreeSpeedMPS + ") for the configured gear ratio");
  }
}
