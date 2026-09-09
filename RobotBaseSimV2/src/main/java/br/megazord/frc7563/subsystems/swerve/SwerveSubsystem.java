// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.DriveConstants.DriveMode;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule fLModule;
  private SwerveModule fRModule;
  private SwerveModule bLModule;
  private SwerveModule bRModule;
  private Gyro gyro;

  private SwerveModule[] modules = new SwerveModule[4];
  private SwerveModuleState[] desiredStates = new SwerveModuleState[4];
  private SwerveModuleState[] states = new SwerveModuleState[4];
  private SwerveModulePosition[] positions = new SwerveModulePosition[4];

  private DriveMode driveMode = DriveMode.SLOW;

  private static SwerveSubsystem instance;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  /**
   * Private constructor for the SwerveSubsystem singleton.
   * 
   * @param fLModule Front-left swerve module.
   * @param fRModule Front-right swerve module.
   * @param bLModule Back-left swerve module.
   * @param bRModule Back-right swerve module.
   * @param gyro     Gyroscope for orientation sensing.
   */
  private SwerveSubsystem(SwerveModule fLModule, SwerveModule fRModule, SwerveModule bLModule, SwerveModule bRModule,
      Gyro gyro) {
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

  public static SwerveSubsystem getInstance(SwerveModule fLModule, SwerveModule fRModule, SwerveModule bLModule,
      SwerveModule bRModule, Gyro gyro) {
    if (instance == null) {
      instance = new SwerveSubsystem(fLModule, fRModule, bLModule, bRModule, gyro);
    }
    return instance;
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    gyro.periodic();

    getModuleStates();
    getModulePositions();
  }

  public void setDriveMode(DriveMode mode) {
    this.driveMode = mode;

    for (int i = 0; i < modules.length; i++) {
      System.out.println("Setting drive mode for module " + i + " to " + mode + "!");
    }
  }

  public double getDriveSpeed() {
    return driveMode.getSpeedValue();
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public Rotation2d getGyroAngle() {
    return gyro.getAngle();
  }
  
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }

    this.positions = positions;
    return positions;
  }

    /**
   * Drive speeds m/s of each module
   * and angle in rad
   * 
   * @return states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    this.states = states;
    return states;
  }

  /**
   * Stop all modules with brake mode. This is used for emergency stops and when the robot is disabled.
   */
  public void stopModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].brake();
    }
  }

   /**
   * Disable all modules with coast mode. This is used for when the robot is disabled and we want to allow the modules to coast to a stop.
   */
  public void disableModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].coast();
    }
  }

    /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {

    // normalize the wheel speeds ;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);

    // Output Module States to each one
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(desiredStates[i]);

    }
    // Store states for logging in
    desiredStates = desiredStates;
  }

   /**
   * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
   * Path Planner uses
   * 
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(chassisSpeeds);

    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Method to get the ROBOT RELATIVE ChassisSpeeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    // Relative to robot
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
  * Gets relative field speed from robot 
  *
  * @return ChassisSpeeds from field relative
  */
  public ChassisSpeeds getRelativeFieldChassisSpeeds()
  {
    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getGyroAngle());
    return speeds;
  }
}
