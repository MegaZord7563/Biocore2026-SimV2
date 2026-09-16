// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import br.megazord.frc7563.Constants.ModuleConstants;
import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link SwerveConversions}.
 *
 * <p>
 * Referenced directly from {@link SwerveConversions}'s class javadoc: these conversions
 * are the only thing standing between wheel rotations (what the drive TalonFX speaks)
 * and meters (what WPILib kinematics/odometry speak). Getting the gear ratio or wheel
 * circumference wrong here silently scales the whole drivetrain - that exact mistake
 * shipped for a season undetected (see the 3.13x note in {@code SwerveConversions}),
 * which is what these tests exist to catch.
 */
class SwerveConversionsTest {

  private static final double DELTA = 1e-9;

  @Test
  void wheelRotationsToMeters_oneRotation_equalsWheelCircumference() {
    assertEquals(
        ModuleConstants.kWheelCircumferenceMeters,
        SwerveConversions.wheelRotationsToMeters(1.0),
        DELTA);
  }

  @Test
  void wheelRotationsToMeters_zeroRotations_isZeroMeters() {
    assertEquals(0.0, SwerveConversions.wheelRotationsToMeters(0.0), DELTA);
  }

  @Test
  void wheelRotationsToMeters_negativeRotations_givesNegativeMeters() {
    assertEquals(
        -ModuleConstants.kWheelCircumferenceMeters,
        SwerveConversions.wheelRotationsToMeters(-1.0),
        DELTA);
  }

  @Test
  void metersToWheelRotations_oneCircumference_equalsOneRotation() {
    assertEquals(
        1.0,
        SwerveConversions.metersToWheelRotations(ModuleConstants.kWheelCircumferenceMeters),
        DELTA);
  }

  @Test
  void metersToWheelRotations_isInverseOf_wheelRotationsToMeters() {
    for (double rotations : new double[] {0.0, 1.0, -1.0, 3.7, -12.25, 150.0}) {
      double meters = SwerveConversions.wheelRotationsToMeters(rotations);
      assertEquals(
          rotations,
          SwerveConversions.metersToWheelRotations(meters),
          DELTA,
          "round trip failed for " + rotations + " wheel rotations");
    }
  }

  @Test
  void wheelRotationsPerSecToMetersPerSec_matchesPositionConversion() {
    // Velocity conversion must use the same circumference as the position conversion -
    // if one gets tuned and the other doesn't, odometry and closed-loop velocity control
    // silently disagree with each other.
    double rotationsPerSec = 16.58; // ~free-spin wheel speed for this drivetrain
    assertEquals(
        SwerveConversions.wheelRotationsToMeters(rotationsPerSec),
        SwerveConversions.wheelRotationsPerSecToMetersPerSec(rotationsPerSec),
        DELTA);
  }

  @Test
  void metersPerSecToWheelRotationsPerSec_isInverseOf_wheelRotationsPerSecToMetersPerSec() {
    for (double metersPerSec : new double[] {0.0, 1.0, -1.0, 4.9, -4.9}) {
      double wheelRotationsPerSec = SwerveConversions.metersPerSecToWheelRotationsPerSec(metersPerSec);
      assertEquals(
          metersPerSec,
          SwerveConversions.wheelRotationsPerSecToMetersPerSec(wheelRotationsPerSec),
          DELTA,
          "round trip failed for " + metersPerSec + " m/s");
    }
  }
}
