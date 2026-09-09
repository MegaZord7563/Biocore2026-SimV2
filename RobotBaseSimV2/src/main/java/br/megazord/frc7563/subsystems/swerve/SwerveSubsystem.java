// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import br.megazord.frc7563.Constants.DriveConstants.DriveMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule fLModule;
  private SwerveModule fRModule;
  private SwerveModule bLModule;
  private SwerveModule bRModule;
  private Gyro gyro;

  private SwerveModule[] modules = new SwerveModule[4];

  private DriveMode driveMode = DriveMode.SLOW;

  private static SwerveSubsystem instance;

  /** Creates a new SwerveSubsystem. */
  private SwerveSubsystem(SwerveModule fLModule, SwerveModule fRModule, SwerveModule bLModule, SwerveModule bRModule, Gyro gyro) {
    this.fLModule = fLModule;
    this.fRModule = fRModule;
    this.bLModule = bLModule;
    this.bRModule = bRModule;
    this.gyro = gyro;

    modules[0] = this.fLModule;
    modules[1] = this.fRModule;
    modules[2] = this.bLModule;
    modules[3] = this.bRModule;
  }

  public static SwerveSubsystem getInstance(SwerveModule fLModule, SwerveModule fRModule, SwerveModule bLModule, SwerveModule bRModule, Gyro gyro) {
    if (instance == null) {
      instance = new SwerveSubsystem(fLModule, fRModule, bLModule, bRModule, gyro);
    }
    return instance;
  }

  @Override
  public void periodic() {
    for(SwerveModule module : modules) {
      module.periodic();
    }

    gyro.periodic();
  }

  public void setDriveMode(DriveMode mode) {
    this.driveMode = mode;

    for(int i = 0; i < modules.length; i++) {
      System.out.println("Setting drive mode for module " + i + " to " + mode + "!");
    }
  }

  public double getDriveSpeed()
  {
    return driveMode.getSpeedValue();
  }

  public DriveMode getDriveMode()
  {
    return driveMode;
  }
}
