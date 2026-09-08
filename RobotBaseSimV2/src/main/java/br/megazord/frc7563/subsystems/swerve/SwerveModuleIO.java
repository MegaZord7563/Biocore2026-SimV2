// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * Interface for swerve module I/O operations.
 * 
 */
public interface SwerveModuleIO 
{
  @AutoLog
  public static class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRads = 0.0;
    public double driveVelocityRadsPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveSupplyCurrentAmps = 0.0;
    public double driveTorqueCurrentAmps = 0.0;
    public double driveTempCelsius;

    public boolean turnConnected = false;
    public Rotation2d turnAbsolutePositionRads = Rotation2d.kZero;
    public Rotation2d turnPositionRads = Rotation2d.kZero;
    public double turnVelocityRadsPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnSupplyCurrentAmps = 0.0;
    public double turnTorqueCurrentAmps = 0.0;
    public double turnTempCelsius;

    public boolean cancoderConnected = false;
    public double chassisAngularOffset = 0.0;
  }

  public static enum SwerveModuleIOOutputMode {
    COAST,
    BRAKE,
    DRIVE,
    CHARACTERIZE
  }

  public static class SwerveModuleIOOutputs {
    public SwerveModuleIOOutputMode mode = SwerveModuleIOOutputMode.COAST;

    public double driveVelocityRadPerSec = 0.0;
    public double driveCharacterizationOutput = 0.0;
    public Rotation2d turnRotation = Rotation2d.kZero;
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  public default void applyOutputs(SwerveModuleIOOutputs outputs) {}

  public default void resetDriveEncoders() {}
}

