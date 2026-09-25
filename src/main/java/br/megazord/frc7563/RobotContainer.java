// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.OIConstants;
import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.commands.shooter.ShootAimTargetCommand;
import br.megazord.frc7563.commands.shooter.TrackTargetTurretActiveCommand;
import br.megazord.frc7563.subsystems.LedSubsystem;
import br.megazord.frc7563.subsystems.feeder.FeederIOSim;
import br.megazord.frc7563.subsystems.feeder.FeederIOTalonFX;
import br.megazord.frc7563.subsystems.feeder.FeederSubsystem;
import br.megazord.frc7563.subsystems.intake.IntakeIOSim;
import br.megazord.frc7563.subsystems.intake.IntakeIOTalonFX;
import br.megazord.frc7563.subsystems.intake.IntakeSubsystem;
import br.megazord.frc7563.subsystems.shooter.Flywheel.FlywheelIOSim;
import br.megazord.frc7563.subsystems.shooter.Flywheel.FlywheelIOTalonFX;
import br.megazord.frc7563.subsystems.shooter.Flywheel.FlywheelSubsystem;
import br.megazord.frc7563.subsystems.shooter.hood.HoodIOSim;
import br.megazord.frc7563.subsystems.shooter.hood.HoodIOTalonFX;
import br.megazord.frc7563.subsystems.shooter.hood.HoodSubsystem;
import br.megazord.frc7563.subsystems.shooter.turret.TurretIOSim;
import br.megazord.frc7563.subsystems.shooter.turret.TurretIOTalonFX;
import br.megazord.frc7563.subsystems.shooter.turret.TurretSubsystem;
import br.megazord.frc7563.subsystems.swerve.Gyro;
import br.megazord.frc7563.subsystems.swerve.GyroIOPygeon2;
import br.megazord.frc7563.subsystems.swerve.GyroIOSim;
import br.megazord.frc7563.subsystems.swerve.SwerveModule;
import br.megazord.frc7563.subsystems.swerve.SwerveModuleIOSim;
import br.megazord.frc7563.subsystems.swerve.SwerveModuleIOTalonFx;
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
  // Subsystems instance
  private SwerveSubsystem swerveDrive;
  private FlywheelSubsystem flywheelSubsystem;
  private TurretSubsystem turretSubsystem;
  private HoodSubsystem hoodSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private FeederSubsystem feederSubsystem;
  public static LedSubsystem ledSubsystem;

  //controllers intace
  public final CommandXboxController driverJoystick = new CommandXboxController(0);

  //commands instance 
  private final TrackTargetTurretActiveCommand trackTargetTurretActiveCommand;
  private final ShootAimTargetCommand shootAimTargetCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (RobotConstants.robotMode) {
      case SIM:
        swerveDrive =
              new SwerveSubsystem(
                  new SwerveModule(new SwerveModuleIOSim(), "FL"),
                  new SwerveModule(new SwerveModuleIOSim(), "FR"),
                  new SwerveModule(new SwerveModuleIOSim(), "BL"),
                  new SwerveModule(new SwerveModuleIOSim(), "BR"),
                  new Gyro(new GyroIOSim(()-> swerveDrive.getAngularVelocity())));

        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOSim());
        turretSubsystem = new TurretSubsystem(new TurretIOSim());
        hoodSubsystem = new HoodSubsystem(new HoodIOSim());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOSim());
        feederSubsystem = new FeederSubsystem(new FeederIOSim());
        break;

      case REAL:
        swerveDrive =
              new SwerveSubsystem(
                  new SwerveModule(new SwerveModuleIOTalonFx(
                    DriveConstants.kFrontLeftDriveMotorPort,
                    DriveConstants.kFrontLeftTurningMotorPort,
                    DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetFLTurning,
                    DriveConstants.kFrontLeftChassisAngularOffset),
                  "FL"),
                  new SwerveModule(new SwerveModuleIOTalonFx(
                    DriveConstants.kFrontRightDriveMotorPort,
                    DriveConstants.kFrontRightTurningMotorPort,
                    DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetFRTurning,
                    DriveConstants.kFrontRightChassisAngularOffset
                  ), 
                  "FR"),
                  new SwerveModule(new SwerveModuleIOTalonFx(
                    DriveConstants.kBackLeftDriveMotorPort,
                    DriveConstants.kBackLeftTurningMotorPort,
                    DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetBLTurning,
                    DriveConstants.kBackLeftChassisAngularOffset
                  ), 
                  "BL"),
                  new SwerveModule(new SwerveModuleIOTalonFx(
                    DriveConstants.kBackRightDriveMotorPort,
                    DriveConstants.kBackRightTurningMotorPort,
                    DriveConstants.kBackRightDriveAbsoluteEncoderPort,
                    DriveConstants.angleOffsetBRTurning,
                    DriveConstants.kBackRightChassisAngularOffset
                  ), 
                  "BR"),
                  new Gyro(new GyroIOPygeon2()));
        flywheelSubsystem = new FlywheelSubsystem(new FlywheelIOTalonFX());
        turretSubsystem = new TurretSubsystem(new TurretIOTalonFX());
        hoodSubsystem = new HoodSubsystem(new HoodIOTalonFX());
        intakeSubsystem = new IntakeSubsystem(new IntakeIOTalonFX());
        feederSubsystem = new FeederSubsystem(new FeederIOTalonFX());
        break;

      default:
        break;
    }

    ledSubsystem = new LedSubsystem(swerveDrive, 0);

    //Commands Instance
    trackTargetTurretActiveCommand = new TrackTargetTurretActiveCommand(turretSubsystem);
    shootAimTargetCommand = new ShootAimTargetCommand(hoodSubsystem, flywheelSubsystem);

    swerveDrive.setDefaultCommand(new RunCommand(
        () -> swerveDrive.driveFieldOriented(
            () -> -MathUtil.applyDeadband(driverJoystick.getLeftY(), OIConstants.kDeadband),
            () -> -MathUtil.applyDeadband(driverJoystick.getLeftX(), OIConstants.kDeadband),
            () -> -MathUtil.applyDeadband(driverJoystick.getRightX(), OIConstants.kDeadband),
            () -> driverJoystick.rightStick().getAsBoolean()),
        swerveDrive)// .onlyIf(()-> !driverJoystick.getHID().getXButton())
    );
    turretSubsystem.setDefaultCommand(trackTargetTurretActiveCommand);

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
    driverJoystick.rightBumper().whileTrue(swerveDrive.setDriveModeCommand(DriveConstants.DriveMode.SLOW)).onFalse(swerveDrive.setDriveModeCommand(DriveConstants.DriveMode.FAST));
    driverJoystick.leftBumper().whileTrue(swerveDrive.setDriveModeCommand(DriveConstants.DriveMode.MAX)).onFalse(swerveDrive.setDriveModeCommand(DriveConstants.DriveMode.FAST));

    driverJoystick.start().onTrue(new InstantCommand(()-> swerveDrive.SeedHeadingCamera()).ignoringDisable(true));

    /** Shooter Calculator **/
    // While held, continuously solves for turret/hood/flywheel setpoints that
    // hit the hub from wherever the robot currently is (and however it's
    // currently moving) and drives the mechanisms to them.
    driverJoystick.rightTrigger(0.5).toggleOnTrue(shootAimTargetCommand).onTrue(new InstantCommand(()-> swerveDrive.setDriveMode(DriveConstants.DriveMode.SHOOTING)));

    /** Intake Commands */
    driverJoystick.leftTrigger(0.5).onTrue(new InstantCommand(()-> intakeSubsystem.intake(), intakeSubsystem)).onFalse(new InstantCommand(()-> intakeSubsystem.setCoastOut()));
    driverJoystick.b().onTrue(new InstantCommand(()-> intakeSubsystem.outtake(), intakeSubsystem)).onFalse(new InstantCommand(()-> intakeSubsystem.setCoastOut(), intakeSubsystem));

    driverJoystick.a().onFalse(new InstantCommand(()-> feederSubsystem.setFeederVoltageOut(12), feederSubsystem)).onFalse(new InstantCommand(()-> feederSubsystem.setCoastOut(), feederSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
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
