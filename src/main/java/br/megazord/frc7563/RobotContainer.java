// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.OIConstants;
import br.megazord.frc7563.subsystems.swerve.Gyro;
import br.megazord.frc7563.subsystems.swerve.GyroIOSim;
import br.megazord.frc7563.subsystems.swerve.SwerveModule;
import br.megazord.frc7563.subsystems.swerve.SwerveModuleIOSim;
import br.megazord.frc7563.subsystems.swerve.SwerveSubsystem;
import edu.wpi.first.math.MathUtil;
// Wpilib imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private SwerveSubsystem swerveDrive;


  private static final CommandXboxController driverJoystick = new CommandXboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerveDrive =
              new SwerveSubsystem(
                  new SwerveModule(new SwerveModuleIOSim(), "FL"),
                  new SwerveModule(new SwerveModuleIOSim(), "FR"),
                  new SwerveModule(new SwerveModuleIOSim(), "BL"),
                  new SwerveModule(new SwerveModuleIOSim(), "BR"),
                  new Gyro(new GyroIOSim(()-> swerveDrive.getAngularVelocity())));

    swerveDrive.setDefaultCommand(new RunCommand(
        () -> swerveDrive.driveFieldOriented(
            () -> -MathUtil.applyDeadband(driverJoystick.getLeftY(), OIConstants.kDeadband),
            () -> -MathUtil.applyDeadband(driverJoystick.getLeftX(), OIConstants.kDeadband),
            () -> -MathUtil.applyDeadband(driverJoystick.getRightX(), OIConstants.kDeadband),
            () -> driverJoystick.rightStick().getAsBoolean()),
        swerveDrive)// .onlyIf(()-> !driverJoystick.getHID().getXButton())
    );
    
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
    /** Swerve Comands **/
    //speed controls
    driverJoystick.rightBumper().onTrue(new InstantCommand(()-> swerveDrive.setDriveMode(DriveConstants.DriveMode.FAST), swerveDrive));
    driverJoystick.leftBumper().onTrue(new InstantCommand(()-> swerveDrive.setDriveMode(DriveConstants.DriveMode.SLOW), swerveDrive));
    driverJoystick.leftBumper().and(driverJoystick.rightBumper()).onTrue(new InstantCommand(()-> swerveDrive.setDriveMode(DriveConstants.DriveMode.MAX), swerveDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  /**
   * Use this to pass the routine test robot systems command to them {@link Robot}
   * class.
   * 
   * @return the command to run in test DS mode
   */
  public Command getRobotTestCommand() {
    return null;
  }
}
