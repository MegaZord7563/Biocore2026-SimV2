// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;

import org.littletonrobotics.junction.LogFileUtil;
// Akit imports 
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
// Wpilib imports 
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// local imports
import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.build.BuildConstants;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Command m_testCommand;

  private final RobotContainer m_robotContainer;

  private double autoStart;
  private boolean autoMessagePrinted;

  private final Timer lowBatteryTimer = new Timer();

  double batteryVoltage = 12.0;
  double lowBatteryVoltageLimit = 11.0;

  private final Alert batteryAlert = new Alert(
      "Battery voltage is very low, please replace the battery.",
      AlertType.kWarning);

  private Alert driverJoystickAlert = new Alert(
      "DriverJoystick on port 0 is not connected. Please connect the joystick and restart the robot.",
      Alert.AlertType.kWarning);

  private Alert arcadeJoyLeftAlert = new Alert(
      "Mesinha on port 1 is not connected. Please connect and restart the robot.",
      Alert.AlertType.kInfo);

  private Alert arcadeJoyRightAlert = new Alert(
      "Mesinha on port 2 is not connected. Please connect and restart the robot.",
      Alert.AlertType.kInfo);

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    super(RobotConstants.loopPeriodSecs);

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    Logger.recordMetadata("RobotVersion", RobotConstants.robotVerion);
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("RobotMode", RobotConstants.robotMode.toString());
    Logger.recordMetadata("BuildVersion", BuildConstants.VERSION);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitRevision", Integer.toString(BuildConstants.GIT_REVISION));
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    String logPath = "/U/logs/";
    String logName;

    if (DriverStation.isFMSAttached()) {
      Logger.recordMetadata("Mode", "FMS");

      Logger.recordMetadata("Event", DriverStation.getEventName());
      Logger.recordMetadata("MatchType", DriverStation.getMatchType().toString());
      Logger.recordMetadata("MatchNumber", Integer.toString(DriverStation.getMatchNumber()));
      Logger.recordMetadata("Alliance", DriverStation.getAlliance().toString());

      logName = String.format(
          "%s_%d_%s.wpilog",
          DriverStation.getMatchType().toString(),
          DriverStation.getMatchNumber(),
          DriverStation.getAlliance().toString());

    } else {
      Logger.recordMetadata("Mode", "TEST");

      String timestamp = java.time.LocalDateTime.now().toString().replace(":", "-");

      Logger.recordMetadata("RunTime", timestamp);

      logName = "TEST_" + timestamp + ".wpilog";
    }

    if (new java.io.File("/U").exists()) {
      logPath = "/U/logs/";
    } else {
      logPath = "/home/lvuser/logs/";
    }

    switch (RobotConstants.robotMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter(logPath + logName));
        // Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter());
        break;

      case REPLAY:
        setUseTiming(false);

        logPath = LogFileUtil.findReplayLog();

        Logger.setReplaySource(
            new WPILOGReader(logPath));

        Logger.addDataReceiver(
            new WPILOGWriter(
                LogFileUtil.addPathSuffix(logPath, "_sim")));

        break;

      default:
        break;
    }

    Logger.registerURCL(URCL.startExternal());
    // StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may
    // be added.
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Print auto duration
    if (m_autonomousCommand != null) {
      if (!m_autonomousCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.printf(
              "*** Auto finished in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        } else {
          System.out.printf(
              "*** Auto cancelled in %.2f secs ***%n", Timer.getTimestamp() - autoStart);
        }
        autoMessagePrinted = true;
      }
    }

    // Get Robot Battery Voltage
    batteryVoltage = RobotController.getBatteryVoltage();

    // Set SmartDashboard inputs
    SmartDashboard.putNumber("Match time", DriverStation.getMatchTime());
    SmartDashboard.putNumber("Batery Voltage", batteryVoltage);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // Log importants game infos and alerts
    Logger.recordOutput("Alerts/BatteryAlert", batteryAlert.get());
    Logger.recordOutput("Alerts/DriverJoystickAlert", driverJoystickAlert.get());
    Logger.recordOutput("Alerts/MesinhaJoy1JoystickAlert", arcadeJoyLeftAlert.get());
    Logger.recordOutput("Alerts/MesinhaJoy2JoystickAlert", arcadeJoyRightAlert.get());
    Logger.recordOutput("Robot/BatteryVoltage", batteryVoltage);
    Logger.recordOutput("Robot/MatchTime", DriverStation.getMatchTime());

    // driverJoystickAlert.set();
    // arcadeJoyLeftAlert.set();
    // arcadeJoyRightAlert.set();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    autoStart = Timer.getTimestamp();
    autoMessagePrinted = false;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    lowBatteryTimer.reset();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // Battery Alert
    if (batteryVoltage > 0
        && batteryVoltage <= lowBatteryVoltageLimit
        && lowBatteryTimer.hasElapsed(3.0)) {
      batteryAlert.set(true);
    }

    if (batteryVoltage > lowBatteryVoltageLimit) {
      lowBatteryTimer.reset();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    m_testCommand = m_robotContainer.getRobotTestCommand();

    // schedule the autonomous command (example)
    if (m_testCommand != null) {
      CommandScheduler.getInstance().schedule(m_testCommand);
    }
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
