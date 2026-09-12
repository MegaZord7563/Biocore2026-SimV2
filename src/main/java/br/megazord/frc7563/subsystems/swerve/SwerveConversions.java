// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import br.megazord.frc7563.Constants.ModuleConstants;

/**
 * Conversions between the units WPILib speaks and the units the drive TalonFX speaks.
 *
 * <p>
 * The drive motor is configured with {@code SensorToMechanismRatio} set to the gear
 * ratio, which makes the WHEEL the mechanism. Everything the motor reports or accepts
 * is therefore in wheel rotations, never meters. WPILib's kinematics, odometry and pose
 * estimator all work in meters. Every crossing of that boundary goes through here.
 *
 * <p>
 * This class exists as its own type so the conversions can be unit tested without a
 * roboRIO or a CAN bus - see {@code SwerveConversionsTest}. Missing these conversions
 * scaled the whole drivetrain by 3.13x and went unnoticed for a season.
 */
public final class SwerveConversions {

  /** Static utility - not instantiable. */
  private SwerveConversions() {
  }

  /**
   * Distance travelled by the wheel.
   *
   * @param wheelRotations turns of the wheel, as reported by {@code getPosition()}
   * @return distance in meters
   */
  public static double wheelRotationsToMeters(double wheelRotations) {
    return wheelRotations * ModuleConstants.kWheelCircumferenceMeters;
  }

  /**
   * Turns of the wheel needed to cover a distance.
   *
   * @param meters distance in meters
   * @return turns of the wheel
   */
  public static double metersToWheelRotations(double meters) {
    return meters / ModuleConstants.kWheelCircumferenceMeters;
  }

  /**
   * Ground speed of the wheel.
   *
   * @param wheelRotationsPerSec wheel rot/s, as reported by {@code getVelocity()}
   * @return speed in meters per second
   */
  public static double wheelRotationsPerSecToMetersPerSec(double wheelRotationsPerSec) {
    return wheelRotationsPerSec * ModuleConstants.kWheelCircumferenceMeters;
  }

  /**
   * Wheel speed to command for a desired ground speed.
   *
   * @param metersPerSec desired ground speed in meters per second
   * @return wheel rotations per second, as {@code VelocityVoltage} expects
   */
  public static double metersPerSecToWheelRotationsPerSec(double metersPerSec) {
    return metersPerSec / ModuleConstants.kWheelCircumferenceMeters;
  }
}
