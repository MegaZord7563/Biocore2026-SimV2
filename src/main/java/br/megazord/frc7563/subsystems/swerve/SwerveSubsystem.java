// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.DriveConstants.DriveMode;
import br.megazord.frc7563.Constants.PathPlannerConstants;
import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.Constants.VisionConstants;
import br.megazord.frc7563.RobotState;
import br.megazord.frc7563.subsystems.vision.VisionCamera;
import br.megazord.frc7563.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule fLModule;
  private SwerveModule fRModule;
  private SwerveModule bLModule;
  private SwerveModule bRModule;
  private Gyro gyro;

  private SwerveModule[] modules = new SwerveModule[4];

  private final RobotState robotState = RobotState.getInstance();

  private SwerveModuleState[] desiredStates = new SwerveModuleState[4];

  private DriveMode driveMode = DriveMode.SLOW;

  /** Limelight seed */
  private boolean allianceSeedHeading = false;
  private double countLL = 0;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  // PathPlanner Config
  private RobotConfig config;
  private Alert pathPlannerNotInitialized = new Alert("PathPlanner has an initialize error! Please restart RobotCode!",
      AlertType.kError);

  /**
   * Private constructor for the SwerveSubsystem singleton.
   * 
   * @param fLModule Front-left swerve module.
   * @param fRModule Front-right swerve module.
   * @param bLModule Back-left swerve module.
   * @param bRModule Back-right swerve module.
   * @param gyro     Gyroscope for orientation sensing.
   */
  public SwerveSubsystem(SwerveModule fLModule, SwerveModule fRModule, SwerveModule bLModule, SwerveModule bRModule,
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

    // Initialize pose estimator (owned by RobotState now) ALWAYS towards to red
    // wall
    robotState.initializePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroAngle(),
        new SwerveModulePosition[] {
            fLModule.getPosition(),
            fRModule.getPosition(),
            bLModule.getPosition(),
            bRModule.getPosition()
        },
        new Pose2d());

    /**
     * Initialize the PathPlanner config and AutoBuilder
     * If the robot's configuration file is missing or corrupted, this code will:
     * Catch the exception thrown by RobotConfig.fromGUISettings().
     * Report the error to the Driver Station, allowing the team to investigate and
     * fix the issue.
     */

    try {
      // config = RobotConfig.fromGUISettings();

      // Manually create the Robot config
      config = PathPlannerConstants.robotConfig;

      AutoBuilder.configure(
          this::getPoseEstimator, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                       // optionally outputs individual module feedforwards
          PathPlannerConstants.AutoConfig,
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              System.out.println("Auto builder ok!!");
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      pathPlannerNotInitialized.set(true);
    }
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    if (DriverStation.isDisabled()) {
      disableModules();
    }

    gyro.periodic();

    getModuleStates();
    getModulePositions();

    updatePoseEstimator();

    if (!allianceSeedHeading && countLL < 100 && !DriverStation.isEnabled()) {
      SeedHeadingAlliance();
      ++countLL;
    }

    robotState.setMeasuredChassisSpeeds(getChassisSpeeds());
  }

  /**
   * Seeds the gyro yaw with the yaw from the Limelight botpose, adjusted for
   * alliance color.
   * 
   * Do NOT TOUCH in this function. Unless you know what you are doing
   * 
   * @return botpose Yaw from limelight
   */
  public void SeedHeadingAlliance() {
    List<VisionCamera> cameras = VisionSubsystem.getInstance().getCameras();

    VisionCamera mainCamera = cameras.stream()
        .filter(VisionCamera::hasSeedPose)
        .filter(c -> c.getSeedTagCount() > 0
            && c.getGlobalAvgTagDistanceMeters() < VisionConstants.kBestPoseMaxTagDistanceMeters)
        .max(Comparator.comparingDouble(VisionCamera::getSeedTagCount))
        .orElse(null);

    if (mainCamera == null)
      return;

    gyro.setPosition(mainCamera.getSeedPose().getRotation());
    updatePoseEstimator();
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

  @AutoLogOutput(key = "Drive/Gyro/Rotation2d")
  public Rotation2d getGyroAngle() {
    return gyro.getAngle();
  }

  public Rotation2d getAngularVelocity() {
    return new Rotation2d(getChassisSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all the
   * modules.
   */
  @AutoLogOutput(key = "Drive/SwervePosition/Measured")
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Drive speeds m/s of each module
   * and angle in rad
   * 
   * @return states
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Get Desired States of each module for logging purposes
   */
  @AutoLogOutput(key = "Drive/SwerveStates/Desired")
  public SwerveModuleState[] getModuleDesiredStates() {
    for (int i = 0; i < modules.length; i++) {
      desiredStates[i] = modules[i].getDesiredState();
    }
    return desiredStates;
  }

  /**
   * Stop all modules with brake mode. This is used for emergency stops and when
   * the robot is disabled.
   */
  public void stopModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].brake();
    }
  }

  /**
   * Disable all modules with coast mode. This is used for when the robot is
   * disabled and we want to allow the modules to coast to a stop.
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
  }

  /**
   * Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
   * Path Planner uses
   * 
   * @param chassisSpeeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.loopPeriodSecs);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(targetSpeeds);

    this.setModuleStates(swerveModuleStates);
  }

  /**
   * Method to get the ROBOT RELATIVE ChassisSpeeds
   */
  @AutoLogOutput(key = "Drive/ChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    // Relative to robot
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets relative field speed from robot
   *
   * @return ChassisSpeeds from field relative
   */
  public ChassisSpeeds getRelativeFieldChassisSpeeds() {
    ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getGyroAngle());
    return speeds;
  }

  public Translation2d getSpeedTranslation() {
    return robotState.getSpeedTranslation();
  }

  public Pose2d getChassisSpeedPose2d() {
    ChassisSpeeds speeds = getRelativeFieldChassisSpeeds();
    return new Pose2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, new Rotation2d(speeds.omegaRadiansPerSecond));
  }

  /**
   * Method drive with joystick
   * The use of these parameters as suppliers to dynamically provide speed and
   * field orientation values.
   * Suppliers are especially useful in cases where you need to calculate values
   * dynamically based on factors that can change over time.
   * The Supplier interface gives you a powerful mechanism to make your FRC robot
   * code more flexible and adaptable.
   * Suppliers are incredibly useful in command-based FRC programming because they
   * allow you to:
   * Decouple Logic: You can separate the logic for calculating drive parameters
   * (speeds, orientation)
   * from the actual drive command. This makes your code cleaner and more
   * maintainable.
   * Dynamic Values: You can easily update the speed and orientation values
   * on-the-fly based on real-time conditions,
   * such as sensor feedback or joystick input.
   * 
   * @param xSpdFunction          Speed of the robot in the x direction (forward).
   * @param ySpdFunction          Speed of the robot in the y direction
   *                              (sideways).
   * @param turningSpdFunction    Angular rate of the robot. rad/s
   * @param fieldOrientedFunction Boolean indicating if speeds are relative to the
   *                              field or to therobot.
   * 
   **/

  public void driveFieldOriented(Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction,
      Supplier<Boolean> joystickButtonFunction) {

    // 1. Get real-time joystick inputs
    double xSpeed = Math.pow(xSpdFunction.get(), 3);
    double ySpeed = Math.pow(ySpdFunction.get(), 3);
    double turningSpeed = turningSpdFunction.get();
    // boolean fieldOriented = fieldOrientedFunction.get();
    boolean joystickButton = joystickButtonFunction.get();

    // 3. Make the driving smoother
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * driveMode.getSpeedValue();
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * driveMode.getSpeedValue();
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond
        * (!joystickButton ? 0.75 : 0.95);

    // 4. Construct desired chassis speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
            ySpeed,
            turningSpeed,
            getGyroAngle()));// Do this if fielOrientation is false

    // 6. Output each module states to wheels
    setModuleStates(swerveModuleStates);// */
  }

  /** Feeds the latest sensor measurements into RobotState's pose estimator. */
  public void updatePoseEstimator() {
    robotState.addOdometryObservation(getGyroAngle(),
        this.getModulePositions()// MNL 10/03/2025
    );
  }

  /*
   * reset the pose Estimator to a new location
   * 
   * 
   * @param pose The pose to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    robotState.resetPose(getGyroAngle(),
        this.getModulePositions(),
        pose);
  }

  /**
   * Get pose estimator Pose2d
   * 
   * @return estimated position
   */
  public Pose2d getPoseEstimator() {
    return robotState.getEstimatedPose();
  }

  /**
   * Pathfinds to a specific pose using PathPlanner.
   * 
   * @param poseSupplier The supplier that provides the target pose.
   * @return selected path command
   */
  public Command pathfindToPose(Supplier<Pose2d> poseSupplier, double maxSpeed, double maxAceleration) {
    PathConstraints telePathConstraints = new PathConstraints(maxSpeed,
        maxAceleration,
        2 * Math.PI,
        3 * Math.PI);
    // return AutoBuilder.pathfindToPose(poseSupplier.get(), telePathConstraints);
    return new DeferredCommand(() -> AutoBuilder.pathfindToPose(poseSupplier.get(),
        telePathConstraints,
        0),
        Set.of(this)).beforeStarting(() -> PathPlannerPath.clearCache());
  }

  /**
   * Follows a path using PathPlanner.
   * This method uses the pathfindThenFollowPath method to load a path from a
   * file.
   * 
   * @param path
   * @return selected path command
   */
  public Command pathfindThenFollow(String path) {
    PathPlannerPath.clearCache();

    try {
      config = PathPlannerConstants.robotConfig;
      PathPlannerPath path2go = PathPlannerPath.fromPathFile(path);
      PathConstraints telePathConstraints = new PathConstraints(1,
          1,
          Math.PI,
          Math.PI);

      return AutoBuilder.pathfindThenFollowPath(path2go, telePathConstraints);
      // new DeferredCommand(()-> AutoBuilder.pathfindThenFollowPath(path2go,
      // telePathConstraints), Set.of(this));

    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Follows a path using PathPlanner.
   * This method uses the PathPlannerPath.fromPathFile() method to load a path
   * from a file.
   * 
   * @param pathName
   * @return selected path command
   */
  public Command followPath(String path) {
    // Clear any existing feedback overrides
    PathPlannerPath.clearCache();
    PPHolonomicDriveController.clearXYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();
    try {

      config = RobotConfig.fromGUISettings();
      PathPlannerPath path2go = PathPlannerPath.fromPathFile(path);

      return AutoBuilder.followPath(path2go);

    } catch (Exception e) {
      // Handle exception as needed
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
      return Commands.none();
    }
  }

  /**
   * Ends the current path following operation.
   */
  public void endPath() {
    // PPHolonomicDriveController.overrideXYFeedback(() -> 0.0, () -> 0.0);
    // Calculate feedback from your custom PID controller

    // (() -> this.getPoseEstimator().getX(), ()-> this.getPoseEstimator().getY());
    PPHolonomicDriveController.overrideRotationFeedback(() -> this.getPoseEstimator().getRotation().getRadians());
    PPHolonomicDriveController.clearXYFeedbackOverride();
    PPHolonomicDriveController.clearRotationFeedbackOverride();

    PathPlannerPath.clearCache();
  }
}