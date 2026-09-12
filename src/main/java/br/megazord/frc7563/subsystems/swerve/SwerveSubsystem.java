// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.DriveConstants.DriveMode;
import br.megazord.frc7563.Constants.PathPlannerConstants;
import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//PathPlanner Imports
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule fLModule;
  private SwerveModule fRModule;
  private SwerveModule bLModule;
  private SwerveModule bRModule;
  private Gyro gyro;

  private SwerveModule[] modules = new SwerveModule[4];

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private SwerveModuleState[] desiredStates = new SwerveModuleState[4];

  private DriveMode driveMode = DriveMode.SLOW;

  private static SwerveSubsystem instance;

  private final SlewRateLimiter xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
  private final SlewRateLimiter turningLimiter = new SlewRateLimiter(
      DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

  /**
   * NetworkTables publisher for Pose2d data.
   * Useful for debugging and visualization in tools like Shuffleboard or custom
   * dashboards.
   * Advantage Scope: Can visualize robot pose in real-time.
   */
  private StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct)
      .publish();

  private StructArrayPublisher<SwerveModuleState> publisherMeasured = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyMeasuredStates", SwerveModuleState.struct)
      .publish();

  private StructArrayPublisher<SwerveModuleState> publisherDesired = NetworkTableInstance.getDefault()
      .getStructArrayTopic("MyDesiredStates", SwerveModuleState.struct)
      .publish();

  private StructPublisher<ChassisSpeeds> publisherChassisSpeeds = NetworkTableInstance.getDefault()
      .getStructTopic("MyChassisSpeeds", ChassisSpeeds.struct)
      .publish();

  private StructPublisher<ChassisSpeeds> publisherFieldChassisSpeeds = NetworkTableInstance.getDefault()
      .getStructTopic("MyChassisSpeedRelativeField", ChassisSpeeds.struct)
      .publish();

  private StructPublisher<Rotation2d> publisherRotation2d = NetworkTableInstance.getDefault()
      .getStructTopic("MyRotation2d", Rotation2d.struct)
      .publish();

  /**
   * Standard deviations for the odometry and vision measurements.
   */
  private static final Matrix<N3, N1> odometryStdDevs = VecBuilder.fill(0.015, 0.015, (10 * Math.PI) / 180);// 5 graus
                                                                                                            // em
                                                                                                            // radianos
                                                                                                            // 5° ×
                                                                                                            // π/180
  private static final Matrix<N3, N1> visionStdDevs = VecBuilder.fill(0.15, 0.15, (5 * Math.PI) / 180);// 5 graus em
                                                                                                       // radianos 5° ×
                                                                                                       // π/180

  // PathPlanner Config
  private RobotConfig config;


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

    // Initialize pose estimator ALWAYS towards to red wall
    m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        getGyroAngle(),
        new SwerveModulePosition[] {
            fLModule.getPosition(),
            fRModule.getPosition(),
            bLModule.getPosition(),
            bRModule.getPosition()
        },
        new Pose2d(),
        odometryStdDevs, // 5 graus em radianos 5° × π/180
        visionStdDevs);// 30 graus em radianos 30° × π/180

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
    }
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

    if(DriverStation.isDisabled())
    {
      disableModules();
      
    }

    gyro.periodic();

    getModuleStates();
    getModulePositions();

    updatePoseEstimator();
    
    // Publish the pose to NetworkTables
    publisher.set(this.getPoseEstimator());

    // Publish the module states to NetworkTables
    publisherMeasured.set(getModuleStates());

    // Publish the desired module states to NetworkTables
    publisherDesired.set(getModuleDesiredStates());

    publisherChassisSpeeds.set(this.getChassisSpeeds());

    publisherRotation2d.set(this.getPoseEstimator().getRotation());

    publisherFieldChassisSpeeds.set(this.getRelativeFieldChassisSpeeds());// this.getRelativeFieldChassisSpeeds());
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

  public  Rotation2d getAngularVelocity()
  {
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
        * (joystickButton ? MathUtil.clamp(driveMode.getSpeedValue() + 0.2, 0, 0.9) : driveMode.getSpeedValue());

    // 4. Construct desired chassis speeds
    var swerveModuleStates = DriveConstants.kDriveKinematics
        .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,
            ySpeed,
            turningSpeed,
            getGyroAngle()));// Do this if fielOrientation is false

    // 6. Output each module states to wheels
    setModuleStates(swerveModuleStates);// */
  }

  /** Updates the field relative position of the robot. */
  public void updatePoseEstimator() 
  {
    // Update the pose estimator with the latest sensor measurements
    m_poseEstimator.update(getGyroAngle(),
        this.getModulePositions()// MNL 10/03/2025
    );
  }

  /*
   * reset the pose Estimator to a new location
   * 
   * 
   * @param pose The pose to set the odometry.
   */
  public void resetOdometry(Pose2d pose) 
  {
    m_poseEstimator.resetPosition(getGyroAngle(),
        this.getModulePositions(), // MNL 10/03/2025
        pose);
  }

  /**
   * Get pose estimator Pose2d
   * 
   * @return estimated position
   */
  public Pose2d getPoseEstimator() 
  {
    return m_poseEstimator.getEstimatedPosition();
  }

}