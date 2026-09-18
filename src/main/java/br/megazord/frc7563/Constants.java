// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563;


//CTRE Imports
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//WPI Imports
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class RobotConstants {
        public static final MODE robotMode = edu.wpi.first.wpilibj.RobotBase.isReal() ? MODE.REAL : MODE.SIM;
        public static final String robotVerion = "AstroSim-2026";

        public enum MODE {
            REAL,
            SIM,
            REPLAY
        }

        public static final double loopPeriodSecs = 0.01;
		public static final boolean enableAlerts = robotMode == MODE.REAL ? true : false;
    }

    public static final class ModuleConstants {

    public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60;

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);// 0.1016m
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;// 0.1016*3.14=

    public static final double kDriveL3 = 6.12;
    public static final double kTurningL3 = 21.42857; // The steering gear ratio of the MK4i is 150/7:1.

    public static final double kDriveL4Mk4 = 5.14;
    public static final double kTurningL4Mk4 = 12.8; // The steering gear ratio of the MK4 is 12.8:1.

    public static final double kDriveR2Mk5n= 6.03;
    public static final double kTurningMk5n = 26.09;
    
    public static final double kDriveMotorGearRatio =  kDriveR2Mk5n; // 1 / 5.14; //L4 L3=1/6.12
    public static final double kTurningMotorGearRatio = kTurningMk5n; // 1 /12.8; // 1/18.0

    //public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    //public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * (2 * Math.PI);

    // public static final double kTurningEncoderPositionFactor =2 * Math.PI; //
    // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    //public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    //public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

    /** No-load free speed of the wheel, in wheel rotations per second. */
    public static final double kDriveWheelFreeSpeedRps = kDrivingMotorFreeSpeedRps / kDriveMotorGearRatio;

    /** Theoretical no-load top ground speed, in meters per second (~5.29 m/s). */
    public static final double kDriveWheelFreeSpeedMPS = kDriveWheelFreeSpeedRps * kWheelCircumferenceMeters;
    //public static final double kDriveRPM = kDriveEncoderRPM2MeterPerSec * 60;
   // public static final double kDriveRPMRatio = kDriveRPM / kDriveMotorGearRatio;
    //public static final double kDriveRPMPi = kDriveRPMRatio / Math.PI;
    //public static final double kDriveRPMSpeed = kDriveRPMPi / kWheelDiameterMeters;

    /**
     * MNL 09/11/2025 Tuner X Constants
     * Steer Gains
     * Ks = 0.1, Kv = 1.59, Ka = 0.0 , kp = 100.0, kD = 0.5, kI = 0.0
     * 
     * TEste 09/23/2025 Units =Rotations
     * Steer Ks Kv Ka Kp Kd
     * Valor médio 0.3064025 1.602225 0.01757525 64.55725 2.529675
     * 
     * Drive ID Ks Kv Ka Kp
     * valor médio 0.221835 0.7482675 0.0498445 0.5347
     */

    // 14,285%
    public static final double turningKS = 0.0; // 0.266;// 0.5;//1.5;//0.1;//<-MNL 11/11/2024 0.1; //0.32; // Add 0.1 V
                                                // output
                                                // to overcome static friction
    public static final double turningKV = 0.0;//0.04284;//calculado // 2.547;// 1.63;//0.12; //1.51; // A velocity target of 1 rps results
                                                // in
                                                // 0.12 V output
    public static final double turningKA = 0.0;// 0.11; //0.27;
    public static final double kPTurning = 75.248;// 350.0;//100 ANTERIOR 0,5 //0,25 ; 0.2142875; 0.175; 0.125; 0.23
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0;// 1.0;//0.5

    // The F parameter should only be set when using a velocity-based PID
    // controller,
    // and should be set to zero otherwise to avoid unwanted behavior.
    // public static final double kTurningFF = 0; //1.0/473.0; //

    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    /**********************************
     * Driving TALON Fx PID settings *
     *********************************/
    /*
     * 
     * /**
     * MNL 09/11/2025 Tuner X Constants
     * Steer Gains
     * Ks = 0.1, Kv = 1.59, Ka = 0.0 , kp = 100.0, kD = 0.5, kI = 0.0
     * 
     * Drive Gains
     * Ks = 0.0, Kv = 0.124, Ka = 0.00 , kp = 0.1, kD = 0.0, kI = 0.0
     * 
     * TEste 09/23/2025 Units =Rotations
     * Steer Ks Kv Ka Kp Kd
     * Valor médio 0.3064025 1.602225 0.01757525 64.55725 2.529675
     * 
     * Drive ID Ks Kv Ka Kp
     * valor médio 0.221835 0.7482675 0.0498445 0.5347
     */
    public static final double driveKS = 0.33852;//0;// 0.2306; ///2;// 0.221835;//0.5;//1.5<-MNL 11/11/2024 0.1; //0.32; // Add
                                           // 0.1 V output
                                           // to overcome static friction
    /**
     * Volts per wheel-rotation-per-second.
     *
     * 0.11914 was characterized at the MOTOR shaft, but SensorToMechanismRatio makes
     * the WHEEL the mechanism, so Slot0 gains must be referenced to the wheel. Scaling
     * by the gear ratio lands on 0.71841 - which is where the older 0.7649 / 0.7482675
     * values in this file's history came from. Kept as a product so it stays correct
     * if the gearbox changes.  MNL/units fix 08/28/2026
     */
    public static final double driveKV = 0.11914 * kDriveMotorGearRatio; // 0.71841
    public static final double driveKA = 0.0;//0.0; // 0.27;

    // 11,69%
    public static final double kPdriving = 0.2;//0.15;//pratic // 1.04; // 0.1;//<--MNL11/11/2024 0.0665;//ANTERIOR 0,0665 ; 0.05872615
                                                // // kP =
                                                // 0.11 An error of 1 rps results in 0.11 V output
    public static final double kIdriving = 0.0;
    public static final double kDdriving = 0.0;

    public static final double kFFdriving = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    /* Swerve Current Limiting */
    // TalonFX
    public static final int driveSupplyLowerLimit = 40; // supply current
    public static final int driveSupplyCurrent = 50; // supply current
    public static final int driveStatorCurrent = 90; // stator current

    public static final double driveCurrentThresholdTime = 1.0;
    public static final boolean driveEnableCurrentLimit = true;

    public static double kDriveClosedLoopRamp = 0.25;

    // SparkMax
    public static final int turningSupplyLowerLimit = 30; // supply current
    public static final int turningSupplyCurrent = 40; // supply current
    public static final int turningStatorCurrent = 60; // stator current

    public static final double turningCurrentThresholdTime = 1.0;
    public static final boolean turningEnableCurrentLimit = true;
    public static double kTurningClosedLoopRamp = 0.25;

    public static final double kTurningEncoderPositionPIDMinInput = 0;
    public static final double kTurningEncoderPositionPIDMaxInput = 2 * Math.PI;

    /* Neutral Modes */
    // public static final IdleMode kTurningMotorIdleMode = IdleMode.kCoast; //
    // template was coast SparkMax
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Coast; // TalonFx
    public static final NeutralModeValue turningNeutralMode = NeutralModeValue.Coast; // TalonFx

    // Spark Slot
    public static final ClosedLoopSlot pidSparkSlot = ClosedLoopSlot.kSlot0;

    // CANivore CANbus
    public static final CANBus swerveCAN = new CANBus("swerve");
  }

  // Drive Constants
  public static final class DriveConstants {

    // --------------------------------------------------------------------------
    // 1. DRIVE MODE ENUM 🚦
    // Defines the named speed settings for teleoperated control.
    // --------------------------------------------------------------------------
    public enum DriveMode {
      SLOW(0.3), // Very low speed for precise maneuvers (e.g., scoring)
      FAST(0.6), // Normal driving speed
      MAX(0.9), // Highest possible speed (e.g., traveling across the field)
      SHOOTING(0.2); // Extremely slow speed for shooting better precision on move

      private double speedValue;

      private DriveMode(double speedValue)
      {
        this.speedValue = speedValue;
      }

      public double getSpeedValue()
      {
        return speedValue;
      }
    }

    public enum LedMode {
      BLUE, // Very low speed for precise maneuvers (e.g., scoring)
      GREEN, // Normal driving speed
      PURPLE, // Highest possible speed (e.g., traveling across the field)
      YELLOW,
      RED,
      RED_WAVE,
      BLUE_WAVE,
      GREEN_WAVE,
      RAINBOW,
      ORANGE_WAVE,
      ORANGE,
      WHITE_WAVE,
      PURPLE_WAVE,
      BLACK,
      PINK,
      WHITE,
      hubActive,
      hubNotActive,
      amoustActive,
      AmoustNotActive // Extremely slow speed for alignment or lifting
    }

    /**************************************
     * Specify the kinematics of our robot*
     ************************************/
    public static final double kTrackWidth = 0.552;// Units.inchesToMeters(21);
    // Distance between right and left wheels
    public static final double kWheelBase = 0.539;// Units.inchesToMeters(21);
    // Distance between front and back wheels
    public static final double kDriveRadius = Math.hypot(kTrackWidth / 2, kWheelBase / 2);

    // Robot physical properties useful for path planning and control
    public static final double totalMassKg = 48;
    public static final double momentOfInertia = 2.5;//4.5464; // IA calculou ->4.5464 kg·m², anterior 6.88;
    public static final double coeficientFriction = 1.2;
    /**
     * Drive reduction PathPlanner's RobotConfig models. Follows whatever the modules
     * actually run rather than naming a gearbox, so swapping kDriveMotorGearRatio
     * carries PathPlanner along with it. Guarded by DriveGainsTest.
     */
    public static final double driveGearRatio = ModuleConstants.kDriveMotorGearRatio; // 6.03:1 MK5n R2

    /**********************************************************************
     * Swerve Drive Object - It specifies the location of each swerve *
     * module on the robot this way the wpi library can construct the *
     * geometry of our robot setup and do all the calculations *
     * 
     * @see Modules Location: FL= +X,+Y; FR= +X,-Y; BL=-X, +Y; BR=-X, -Y,*
     **********************************************************************/
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // + - antes
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // + + antes
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // - - antes
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2) // - + antes
    );

    //
    /**
     * MNL 17/10/2024
     * kFrontLeftChassisAngularOffset = 0: This means that when your front left
     * module's turning motor is at its zero position,
     * the module is pointing straight forward relative to the chassis.
     * kFrontRightChassisAngularOffset = 0: This suggests your front right module is
     * also pointing forward when its turning motor
     * is at zero.
     * kBackLeftChassisAngularOffset = Math.PI: This means that your back left
     * module is pointing 180 degrees from the front left
     * module. So, when its turning motor is at zero, the module is pointing
     * straight backward relative to the chassis.
     * kBackRightChassisAngularOffset = Math.PI: This means that your back right
     * module is also pointing straight backward
     * when its turning motor is at zero.
     *****/

    public static final double kFrontLeftChassisAngularOffset = 0;//
    public static final double kFrontRightChassisAngularOffset = Math.PI;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = Math.PI;

    // Portas dos Kraken
    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kBackLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 9;
    public static final int kBackRightDriveMotorPort = 12;

    // Portas dos Sparks
    public static final int kFrontLeftTurningMotorPort = 4;
    public static final int kBackLeftTurningMotorPort = 7;
    public static final int kFrontRightTurningMotorPort = 10;
    public static final int kBackRightTurningMotorPort = 13;

    // CanCoder ports
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 5;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 8;
    public static final int kBackRightDriveAbsoluteEncoderPort = 11;
    public static final int kPigeonPort = 30;// 7563

    /**
     * SINTONIA DAS RODAS
     * FL => FRONT LEFT
     * FR => FRONT RIGHT
     * BL => BACK LEFT
     * BR => BACK RIGHT
     */

    public static final Rotation2d angleOffsetFLTurning = Rotation2d.fromDegrees(109.863281255);// -4.85
    public static final Rotation2d angleOffsetFRTurning = Rotation2d.fromDegrees(-92.548828125);// -77.43, -77.34375
    public static final Rotation2d angleOffsetBLTurning = Rotation2d.fromDegrees(30.9375);// 142.07
    public static final Rotation2d angleOffsetBRTurning = Rotation2d.fromDegrees(81.474609375);// -91.14

    /*******************************************************
     * constante que limita a velocidade maxima drive teleop*
     ******************************************************/

    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.9; // kRAKEN - GEAR RATIO L3
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;// 2*2* Math.PI; //

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;// anterior 1.052,
                                                                                                    // 1.175
                                                                                                    // 4,25531914893617
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 2;// anterior
                                                                                                                        // 2

    // Slew Rate adjustments
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;// anterior 2 - 2/12/2024 MNL
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;// anterior 2 2/12/2024 MNL

    // it's not currently used
    /*
     * public static final double kDirectionSlewRate = 1.2; // radians per second
     * public static final double kMagnitudeSlewRate = 1.8; // percent per second (1
     * = 100%)
     * public static final double kRotationalSlewRate = 2.0; // percent per second
     * (1 = 100%)
     */

    public static final String limelightFront = "limelight-one";
    public static final String limelightBack = "limelight-four";
    public static final String limelightLeft = "limelight-two";
    public static final String limelightRight = "limelight-three";
  }

  /*************************************************************
   * constante que limita a velocidade maxima drive autonomous *
   *************************************************************/
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.0;//DriveConstants.kPhysicalMaxSpeedMetersPerSecond ; // 4
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; // 3
    
   
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond;// div/2//10,20

    
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 3*Math.PI ; // pi/2//4

    public static final double kPXController = 2.0;// 1.5
    public static final double kPYController = 2.0;// 1.5
    public static final double kPThetaController = 2.0;

    public static final double kOffset = Units.inchesToMeters(9);
    public static final double kOffsetSide = Units.inchesToMeters(3);

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(
                                          kMaxAngularSpeedRadiansPerSecond,
                                          kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  /**
   * Path Planner Constants
   */
  public static final class PathPlannerConstants {
    public static final double kPTranslationPath = 3.0;// 3.5 Anterior 0.125,10
    public static final double kITranslationPath = 0.0;
    public static final double kDTranslationPath = 0.0;

    public static final double kPRotationPath = 1.8;// 2.0, 3.0; //3.0; 1.6
    public static final double kIRotationPath = 0.0;
    public static final double kDRotationPath = 0.000;

    public static final PIDConstants kPIDRotationPath = new PIDConstants(kPRotationPath, kIRotationPath,
        kDRotationPath);
    public static final PIDConstants kPIDTranslationPath = new PIDConstants(kPTranslationPath, kITranslationPath,
        kDTranslationPath);

    public static final PPHolonomicDriveController AutoConfig = new PPHolonomicDriveController(kPIDTranslationPath,
        kPIDRotationPath);

    public static final double maxAccelerationPath = 1;// 1.75//1.0//5.0 //3.0
    public static final double maxAngularVelocityRadPerSec = Units.degreesToRadians(360);
    public static final double maxAngularAccelerationRadPerSecSq = Units.degreesToRadians(540);

    public static RobotConfig robotConfig = new RobotConfig(DriveConstants.totalMassKg, // The mass of the robot,
                                                                                        // including bumpers and
                                                                                        // battery, in Kilograms.
        DriveConstants.momentOfInertia, // IA calculou ->4.5464 kg·m², anterior 6.88
        new ModuleConfig(ModuleConstants.kWheelDiameterMeters / 2,
            DriveConstants.kPhysicalMaxSpeedMetersPerSecond,
            DriveConstants.coeficientFriction,
            DCMotor.getKrakenX60(1)
                .withReduction(DriveConstants.driveGearRatio),
            ModuleConstants.driveSupplyCurrent,
            1),
        DriveConstants.kDriveKinematics.getModules());

  }

  /*
   * Subsystems Constants
   * 
   * implement here constants for subsystems
   */
  public static final class SubsystemsConstants {
    public static final class IntakeConstants {
      /* Intake Constants */
      public static final int kIntakeMotorID = 21;

      public static final boolean kIntakeMotorEnableCurrentLimit = true;
      public static final double kIntakeMotorGearRatio = 1;
      public static final double kIntakeClosedLoopRamp = 0;
      public static final NeutralModeValue kIntakeMotorNeutalMode = NeutralModeValue.Coast;
      public static final double kIntakeMotorThresholdCurrent = 120;
      public static final double kIntakeMotorSupplyCurrent = 70;
      // ----------------*Pid Constants Intake rollers*----------------
      public static final double KSIntake = 0.41978;
      public static final double KVIntake = 0.043145;
      public static final double kPIntake = 0.068582;
      public static final double kDIntake = 0;// 0.2
      public static final double kIIntake = 0;// 0.1

      /* Ariculator Constants */
      public static final int kArticulatorMotorID = 20;

      public static final NeutralModeValue kArticulatorMotorNeutalMode = NeutralModeValue.Coast;
      public static final double kArticulatorMotorThresholdCurrent = 80;// 80;

      public static final double kArticulatorMotorSupplyCurrent = 30;
      public static final boolean kArticulatorMotorEnableCurrentLimit = true;
      public static final double kArticulatorMotorGearRatio = 8.57;//<--new gearbox ratio 04/25/2026 9 on the gearbox plus 21/20 pulley ----15.0;
      public static final double kArticulatorClosedLoopRamp = 0.25;

      // ------------------- PID SLOT 0 GAINS-----------------
      public static final double KSArticulator = 1.4756;
      public static final double KVArticulator = 0;
      public static final double kPArticulator = 30;// 6.9823
      public static final double kIArticulator = 0.0;
      public static final double kDArticulator = 5.0526;
 
      // -------------------PID SLOT 1 GAINS-----------------
      public static final double KS1Articulator = 1.4756;
      public static final double KV1Articulator = 0;
      public static final double kP1Articulator = 160;// 6.9823
      public static final double kI1Articulator = 0.0;
      public static final double kD1Articulator = 5.0526;

      public static final double articulatorUpAngle = 0.4;
      public static final double articulatorDownAngle = 2.05;

      public static final double IntakeModeSpeed = 100;

      public static final double OutakeModeSpeed = -50;

      public static final double articulatorStartAngle = 2.15;
    }

    public static final class shooterConstants {
      public static final class ShooterGeometryConstants 
      {

        /* shooter pose on robot */
        public static final double Dx = -0.1445;
        public static final double Dy = -0.1516;
        public static final double Dz = 0.364;
        /******************************
         * Geometry Importants Values *
         ******************************/
        public static final Transform3d RobotToTurret3d = new Transform3d(new Translation3d(Dx, Dy, Dz),
                                                                          Rotation3d.kZero);
        public static final Transform2d RobotToTurret2d = new Transform2d(new Translation2d(Dx, Dy),
                                                                          new Rotation2d(0.0));
      }

      public static final class ShootCalculatorConstansts 
      {
        public static InterpolatingDoubleTreeMap ToFMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap HoodAngleMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap FlywheelMap = new InterpolatingDoubleTreeMap();

        public static InterpolatingDoubleTreeMap getTofmap() 
        {
          return ToFMap;
        }

        public static InterpolatingDoubleTreeMap getHoodanglemap() 
        {
          return HoodAngleMap;
        }

        public static InterpolatingDoubleTreeMap getFlywheelmap() 
        {
          return FlywheelMap;
        }

        public static final double minDistance = 1.34;
        public static final double maxDistance = 5.60;

        static 
        {

          HoodAngleMap.put(1.0, 28.3);
          HoodAngleMap.put(2.0, 28.3 * 2);
          HoodAngleMap.put(3.0, 28.3 * 3);
          HoodAngleMap.put(4.0, 28.3 * 4);
          HoodAngleMap.put(5.0, 28.3 * 5);
          HoodAngleMap.put(6.0, 160.0);

          FlywheelMap.put(1.0, 48.2);
          FlywheelMap.put(1.5, 51.5);
          FlywheelMap.put(2.0, 53.6);
          FlywheelMap.put(2.5, 56.8);
          FlywheelMap.put(3.0, 58.2);
          FlywheelMap.put(3.5, 62.5);
          //FlywheelMap.put(3.8, 63.5);
          FlywheelMap.put(4.00, 66.00);
          FlywheelMap.put(4.24, 63.66);
          FlywheelMap.put(4.30, 63.10);
          FlywheelMap.put(5.42, 71.38);
          FlywheelMap.put(5.76, 71.67);

          // ToFMap.put(1.61, 0.98);
          // ToFMap.put(2.0, 0.97);
          /*ToFMap.put(1.0, 1.05); // 0.92
          ToFMap.put(1.5, 1.0);
          ToFMap.put(2.0, 1.08);
          ToFMap.put(2.5, 1.09);
          ToFMap.put(3.0, 1.15);
          ToFMap.put(4.0, 1.15);
          ToFMap.put(5.0, 1.15);*/
          ToFMap.put(1.0, 1.05);
          ToFMap.put(1.5, 1.00);
          ToFMap.put(2.0, 1.08);
          ToFMap.put(2.5, 1.09);
          ToFMap.put(3.0, 1.15);
          ToFMap.put(3.5, 1.08);
          ToFMap.put(4.0, 1.19);
          ToFMap.put(4.5, 1.25);
          ToFMap.put(5.0, 1.24);
          ToFMap.put(5.5, 1.26);
          ToFMap.put(6.0, 1.24);
         



          /**
           * 
           * 
           * timeOfFlightMap.put(5.68, 1.16);
           * timeOfFlightMap.put(4.55, 1.12);
           * timeOfFlightMap.put(3.15, 1.11);
           * timeOfFlightMap.put(1.88, 1.09);
           * timeOfFlightMap.put(1.38, 0.90);
           */
        }

      }

      public static final class TurretConstants 
      {
        /* to implement */
        /* motors infos */
        public static final int kTurretMotorId = 16;
        public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;

        /* current limit */
        public static final double kMotorThresholdCurrent = 60;
        public static final boolean kMotorEnableCurrentLimit = true;
        /* feedback sensor */
        public static final double kMotorGearRatio = 18.45;//12.3; //18.45;
        public static final boolean kContinuousWrap = false;

        /*----------------- PID constants--------------- */
        public static final double kDriveClosedLoopRamp = 0.25;
        public static final double KS = 0.0;//1.0069;
        public static final double KV = 0.0;//0.0035301;
        public static final double kP = 1000;// 65.619;
        public static final double kI = 0;
        public static final double kD = 0;

        /* soft limits configs */

        public static final boolean kForwardSoftLimitEnable = true;
        public static final boolean kReverseSoftLimitEnable = true;
        public static final double kForwardSoftLimitThreshold = Units.degreesToRotations(180);
        public static final double kReverseSoftLimitThreshold = -Units.degreesToRotations(180);

      }

      public static final class CapoArticulatorConstants 
      {
        /* to implement */
        /* motors infos */

        public static final int kCapoArticulatorMotorId = 18;
        public static final NeutralModeValue kMotorNeutalMode = NeutralModeValue.Coast;
        public static final MotorType kMotorType = MotorType.kBrushless;

        /* current limit */
        public static final double kMotorThresholdCurrent = 60;
        public static final boolean kMotorEnableCurrentLimit = true;

        /* feedback sensor */
        public static final double kDriveMotorGearRatio = 2.833;
        public static final boolean kContinuousWrap = false;

        /* -----------PID constants-------------------- */
        public static final double KS = 0;
        public static final double KV = 0.00566;
        public static final double kP = 70;
        public static final double kI = 0;
        public static final double kD = 0;

        /* soft limits */
        public static final boolean kForwardSoftLimitEnable = true;
        public static final boolean kReverseSoftLimitEnable = false;
        public static final double kForwardSoftLimitThreshold = Units.degreesToRotations(160);
        public static final double kReverseSoftLimitThreshold = 0;
      }

      public static final class FlywheelConstants {

        public static final int kFollowerMotorId = 19;
        /* to implement */
        public static final int kShooterWheelMotorId = 17;
        public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;
        public static final double kMotorThresholdCurrent = 120;
        public static final double kMotorSupplyCurrent = 70;

        public static final boolean kMotorEnableCurrentLimit = true;
        public static final boolean kContinuousWrap = false;
        public static final double kMotorGearRatio = 1;

        // *------------PID GAINS--------------------- */
        public static final double kSlot0kS = 0.23121;//0.32743;
        public static final double kSlot0kV = 0.12235;//0.12664;
        public static final double kSlot0kP = 8.14487;//0.14487;//0.20002;
        public static final double kSlot0kI = 0;
        public static final double kSlot0kD = 0;


        public static final double kSlot1kS = 0.23121;
        public static final double kSlot1kV = 0.12235;
        public static final double kSlot1kP = 0.14487;
        public static final double kSlot1kI = 0;
        public static final double kSlot1kD = 0;
      }
    }

    public static final class FeederConstants {
      public static final int motorFeederPort = 15;
      public static final int motorFeederFollowPort = 22;
      public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Coast;
      public static final double kMotorThresholdCurrent = 120;
      public static final double kMotorSupplyCurrent = 70;
      public static final boolean kMotorEnableCurrentLimit = true;
      public static final boolean kContinuousWrap = false;

      /*--------------PID GAINS---------------- */
      public static final double kSlot0kS = 0.41674;
      public static final double kSlot0kV = 0.04235;
      public static final double kSlot0kP = 0.065389;
      public static final double kSlot0kI = 0;
      public static final double kSlot0kD = 0;
      public static final double kMotorGearRatio = 2.0;
      public static final double kFeederRunRPS = 100;
    }

    public static final class IndexerConstants {
      public static final int motorIndexerPort = 14;
      public static final NeutralModeValue kMotorNeutralMode = NeutralModeValue.Brake;
      public static final double kMotorThresholdCurrent = 60;
      public static final boolean kMotorEnableCurrentLimit = true;
      public static final boolean kContinuousWrap = false;

      /** ----------------PID GAINS----------------- */
      public static final double kSlot0kS = 0.47179;
      public static final double kSlot0kV = 0.043922;
      public static final double kSlot0kP = 0.10941;
      public static final double kSlot0kI = 0;
      public static final double kSlot0kD = 0;

      public static final double kMotorGearRatio = 1;
      public static final double kIndexerRunRPS = 50;
    }
  }

  /* OI Constants */
  public static final class OIConstants {
    /*
     * Joystick Driver Constants
     */
    public static final class JoystickDriverConstants {
      // Joystick Driver
      public static final int kDriverControllerPort = 0;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      // public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.1;// 0.05 anterior
      // public static final int kRumbleOn = 1;
      // public static final int kRumbleOff = 0;
    }

    /*
     * Joystick Operator Constants
     */
    public static final class JoystickOperatorConstants {
      // Joystick Provisory Operator
      public static final int kOperatorControllerPort = 1;

      public static final int kDriverYAxis = 1;
      public static final int kDriverXAxis = 0;
      // public static final int kDriverRotAxis = 4;
      public static final int kDriverFieldOrientedButtonIdx = 1;

      public static final double kDeadband = 0.1;// 0.05 anterior
    }

    /*
     * /public static final class JoystickOperatorConstants
     * {
     * //Joystick Operator
     * public static final int kOperatorControllerPort = 1;
     * public static final double kOperatorDeathBand = 0.05;
     * public static final int kX = 1;
     * public static final int kA = 2;
     * public static final int kB = 3;
     * public static final int kY = 4;
     * public static final int kLeftBumper = 5;
     * public static final int kRightBumper = 6;
     * public static final int kLeftTrigger = 7;
     * public static final int kRightTrigger = 8;
     * public static final int kBack = 9;
     * public static final int kStart = 10;
     * public static final int kLeftStick = 11;
     * public static final int kRightStick = 12;
     * }
     */

    /*
     * Mesinha Joy Constants
     */
    public static final class MesinhaJoy1Constants {
      // First part of operator Hub
      public static final int kJoy1Port = 1;
      public static final double kOperatorDeathBand = 0.05;

      public static final int kOne = 1;
      public static final int kTwo = 2;
      public static final int kThree = 3;
      public static final int kFour = 4;
      public static final int kFive = 5;
      public static final int kSix = 6; // ok
      public static final int kSeven = 7; // ok
      public static final int kEight = 8; // ok
      public static final int kNine = 9; // ok
      public static final int kTen = 10; // ok
      public static final int kEleven = 11; // ok
      public static final int kTwelve = 12; // ok
      public static final int kBlackAxisX = 0; // ok
      public static final int kBlackAxisY = 1; // ok
    }

    /**
     * Mesinha Joy2 Constants
     */
    public static final class MesinhaJoy2Constants {
      // Second part of operator Hub
      public static final int kJoy2Port = 2;
      public static final double kOperatorDeathBand = 0.05;

      public static final int kOne = 1;
      public static final int kTwo = 2; // ok
      public static final int kThree = 3; // ok
      public static final int kFour = 4; // ok
      public static final int kFive = 5; // ok
      public static final int kSix = 6; // ok

      public static final int kSeven = 7; // ok
      public static final int kEight = 8;
      public static final int kNine = 9;
      public static final int kTen = 10;
      public static final int kEleven = 11;
      public static final int kTwelve = 12;
      public static final int kRedAxisX = 0; // ok
      public static final int kRedAxisY = 1; // ok

    }

    public static final double kDeadband = 0.1;// 0.1
  }

  /*******************************************************************
   * @param https://docs.revrobotics.com/brushless/neo/v1.1/neo-v1
   * 
   *****************************************************************/

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  /***********************************************************************************************************************************
   * @param https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
   * 
   * @param https://store.ctr-electronics.com/announcing-kraken-x60/?srsltid=AfmBOorh3sPSXQ-WmuWYeJlxrIkATC1wRVPc0V65woNtynzQ1Sil1Ueh
   *
   *************************************************************************************************************************************/
  public static final class KrakenMotorConstants {
    public static final double kFreeSpeedRpm = 6000;
  }

  /**
   * Holonomic Drive Controller Configuration
   * 
   * @param kpTranslation          translation proportional gain
   * @param kITranslation          translation integral gain
   * @param kDTranslation          translation derivative gain
   * @param kPRotation             rotation proportional gain
   * @param kIRotation             rotation integral gain
   * @param kDRotation             rotation derivative gain
   * @param kPTheta                trapezoid profiled theta proportional gain
   * @param kITheta                trapezioid profiled theta integral gain
   * @param kDTheta                trapezioid profiled theta derivative gain
   * @param maxAngularVelocity     trapezoid constraints max angular velocity
   *                               rad/s
   * @param maxAngularAcceleration trapezoid constraints max angular acceleration
   *                               rad/s²
   * @return new HolonomicDriveController
   */
  public static HolonomicDriveController HolonomicControllerConfig(double kpTranslation,
      double kITranslation,
      double kDTranslation,
      double kPRotation,
      double kIRotation,
      double kDRotation,
      double kPTheta,
      double kITheta,
      double kDTheta,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    // MNL 09/26/2025 - HolonomicDriveController need to try out

    return new HolonomicDriveController(
        new PIDController(kpTranslation, kITranslation, kDTranslation), // translation constants
        new PIDController(kPRotation, kIRotation, kDRotation), // rotatoin constants
        new ProfiledPIDController(kPTheta, kITheta, kDTheta,
            // trapezoidal profiled PID controller

            // Here, our rotation profile constraints were a max velocity
            // of 1 rotation per second and a max acceleration of 180 degrees
            // per second squared.
            new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration), 0.02) // max angular velocity
                                                                                                // and acceleration
    ); // Rotation PID constants)
  }
}
