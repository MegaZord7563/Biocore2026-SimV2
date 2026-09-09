// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  SwerveModule fLModule;
  SwerveModule fRModule;
  SwerveModule bLModule;
  SwerveModule bRModule;
  Gyro gyro;

  SwerveModule[] modules = new SwerveModule[4];

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(SwerveModule fLModule, SwerveModule fRModule, SwerveModule bLModule, SwerveModule bRModule, Gyro gyro) {
    this.fLModule = fLModule;
    this.fRModule = fRModule;
    this.bLModule = bLModule;
    this.bRModule = bRModule;
    this.gyro = gyro;

    modules[0] = fLModule;
    modules[1] = fRModule;
    modules[2] = bLModule;
    modules[3] = bRModule;
  }

  @Override
  public void periodic() {
    for(SwerveModule module : modules) {
      module.periodic();
    }

    gyro.periodic();
  }
}
