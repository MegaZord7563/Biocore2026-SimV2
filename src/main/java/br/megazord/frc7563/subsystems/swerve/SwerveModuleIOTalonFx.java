// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import br.megazord.frc7563.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class SwerveModuleIOTalonFx implements SwerveModuleIO {
    // Declare motor Dirves
    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final CANcoder absoluteCaNcoder;

    // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage drivePID = new VelocityVoltage(0).withSlot(0);

    // create a position closed-loop request, voltage output, slot 0 configs
    private final PositionVoltage turnPID = new PositionVoltage(0).withSlot(0);

    /* Keep a brake request so we can disable the motor */
    private final NeutralOut brakeOut = new NeutralOut();

    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private SwerveModuleIOOutputMode mode = SwerveModuleIOOutputMode.COAST;

    private double chassisAngularOffset = 0.0;
    private Rotation2d absoluteEncoderOffset = Rotation2d.kZero;

    public SwerveModuleIOTalonFx(
            int driveMotorId,
            int turnMotorId,
            int absoluteEncoderId,
            Rotation2d absoluteEncoderOffset,
            double chassisAngularOffset) {
        // Instantiate objects
        CANBus swerveCAN = ModuleConstants.swerveCAN;
        this.driveMotor = new TalonFX(driveMotorId, swerveCAN);
        this.turnMotor = new TalonFX(turnMotorId, swerveCAN);
        this.absoluteCaNcoder = new CANcoder(absoluteEncoderId, swerveCAN);

        // Offset config
        this.chassisAngularOffset = chassisAngularOffset;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // initialize absolute encoder
        initializeCanCoderConfigs();
        // Initialize turning motor
        initializeTurningMotor();
        // Initialize drive motor
        initializeDriveMotor();

        //reset to absolute
        resetToAbsolute();
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveMotor.setControl(drivePID);
        } else if (mode == SwerveModuleIOOutputMode.BRAKE) {
            driveMotor.setControl(brakeOut);
        } else {
            driveMotor.setControl(coastOut);
        }
        if (turnClosedLoop) {
            turnMotor.setControl(turnPID);
        } else if (mode == SwerveModuleIOOutputMode.BRAKE) {
            turnMotor.setControl(brakeOut);
        } else {
            turnMotor.setControl(coastOut);
        }

        inputs.driveConnected = driveMotor.isConnected();
        inputs.drivePositionRads = Units.rotationsToRadians(driveMotor.getPosition().getValueAsDouble());
        inputs.driveVelocityRadsPerSec = Units.rotationsToRadians(driveMotor.getVelocity().getValueAsDouble());
        inputs.driveAppliedVolts = driveMotor.getMotorVoltage().getValueAsDouble();
        inputs.driveSupplyCurrentAmps = Math.abs(driveMotor.getSupplyCurrent().getValueAsDouble());
        inputs.driveTempCelsius = driveMotor.getDeviceTemp().getValueAsDouble();

        inputs.turnConnected = turnMotor.isConnected();
        inputs.turnPositionRads = Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble());
        inputs.turnAbsolutePositionRads = Rotation2d
                .fromRotations(absoluteCaNcoder.getAbsolutePosition().getValueAsDouble());
        inputs.turnSupplyCurrentAmps = Math.abs(turnMotor.getSupplyCurrent().getValueAsDouble());
        inputs.turnAppliedVolts = turnMotor.getMotorVoltage().getValueAsDouble();
        inputs.turnTempCelsius = turnMotor.getDeviceTemp().getValueAsDouble();
        inputs.turnVelocityRadsPerSec = Units.rotationsToRadians(turnMotor.getVelocity().getValueAsDouble());

        inputs.chassisAngularOffset = chassisAngularOffset;
        inputs.cancoderConnected = absoluteCaNcoder.isConnected();
    }

    @Override
    public void applyOutputs(SwerveModuleIOOutputs outputs) {
        mode = outputs.mode;
        switch (outputs.mode) {
            case COAST, BRAKE:
                driveClosedLoop = false;
                turnClosedLoop = false;
                break;

            case DRIVE:
                driveClosedLoop = true;
                turnClosedLoop = true;
                drivePID.withVelocity(Units.radiansToRotations(outputs.driveVelocityRadPerSec));
                turnPID.withPosition(Units.radiansToRotations(outputs.turnRotation.getRadians()));
                break;

            case CHARACTERIZE:
                driveClosedLoop = false;
                turnClosedLoop = true;
                turnPID.withPosition(Units.radiansToRotations(outputs.turnRotation.getRadians()));
        }
    }

    @Override
    public void resetDriveEncoders() {
        driveMotor.setPosition(0.0);
    }

    /**
     * @category CanCoder configutation
     */
    private void initializeCanCoderConfigs() {
        absoluteCaNcoder.clearStickyFaults();
        absoluteCaNcoder.getConfigurator().apply(new CANcoderConfiguration());

        var configs = new CANcoderConfiguration()
                .withMagnetSensor(new MagnetSensorConfigs()
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                        .withMagnetOffset(0));

        /*
         * User can change the configs if they want, or leave it empty for
         * factory-default
         */
        absoluteCaNcoder.getConfigurator().apply(configs);

        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(200, absoluteCaNcoder.getPosition(),
                absoluteCaNcoder.getAbsolutePosition(),
                absoluteCaNcoder.getVelocity());
    }

    /**
     * https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
     * Config drive motors
     * 
     * @category TalonFx settings for drive motors
     */
    private void initializeDriveMotor() {
        driveMotor.clearStickyFaults();
        // Factory Default
        this.driveMotor.getConfigurator().apply(new TalonFXConfiguration());
        // driveMotor.setInverted(driveMotorReversed);
        driveMotor.setNeutralMode(ModuleConstants.driveNeutralMode);

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration()
                // Current Limits
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(ModuleConstants.driveSupplyCurrent)// max current drawn from battery 60A
                        .withSupplyCurrentLimitEnable(ModuleConstants.driveEnableCurrentLimit)
                        .withSupplyCurrentLowerLimit(ModuleConstants.driveSupplyLowerLimit) // min current 40A
                        .withSupplyCurrentLowerTime(ModuleConstants.driveCurrentThresholdTime) // after 1 second at min
                                                                                               // current, enable limit
                        .withStatorCurrentLimit(ModuleConstants.driveStatorCurrent) // max current for torque production
                                                                                    // 120A
                        .withStatorCurrentLimitEnable(ModuleConstants.driveEnableCurrentLimit))

                // Feedback Sensor
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(ModuleConstants.kDriveMotorGearRatio)) // 1:1 ratio between sensor
                                                                                           // and mechanism

                // Closed Loop General
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                        .withContinuousWrap(false))

                // PID Slot0
                .withSlot0(new Slot0Configs()
                        .withKV(ModuleConstants.driveKV) 
                        .withKS(ModuleConstants.driveKS) 
                        .withKP(ModuleConstants.kPdriving) // An error of 1 rps results in 0.11 V output
                        .withKI(ModuleConstants.kIdriving) // no output for integrated error
                        .withKD(ModuleConstants.kDdriving)) // no output for error derivative
                // Closed Loop Ramps
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(ModuleConstants.kDriveClosedLoopRamp))

                /// * Tested - voltage compensation configs MNL 04/24/2026
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage(12.2)
                        .withPeakReverseVoltage(-12.2))

                // Motor Output
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(ModuleConstants.driveNeutralMode)
                        .withInverted(InvertedValue.Clockwise_Positive));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = driveMotor.getConfigurator().apply(toConfigure);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply swerve module configs, error code: " + status.toString());
        }

        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(100, driveMotor.getPosition(), driveMotor.getVelocity());
        /* And initialize encoder position to 0 */
        driveMotor.setPosition(0);
    }

    /**
     * Config turning motors
     * 
     * @category TalonFx settings for turning motors
     */
    private void initializeTurningMotor() {
        // Clear faults
        turnMotor.clearStickyFaults();
        // Factory Default
        this.turnMotor.getConfigurator().apply(new TalonFXConfiguration());

        //
        turnMotor.setNeutralMode(ModuleConstants.turningNeutralMode);// always coast

        /* Configure a stator limit of 20 amps */
        TalonFXConfiguration toConfigure = new TalonFXConfiguration()

                // New CTRE 2025 method MNL09/24/2025 testar
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(ModuleConstants.turningStatorCurrent)
                        .withStatorCurrentLimitEnable(ModuleConstants.turningEnableCurrentLimit)
                        .withSupplyCurrentLowerLimit(ModuleConstants.turningSupplyLowerLimit) // min current 40A
                        .withSupplyCurrentLowerTime(ModuleConstants.turningCurrentThresholdTime)) // after 1 second at
                                                                                                  // min current, enable
                                                                                                  // limit

                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(ModuleConstants.kTurningMotorGearRatio))

                // Closed Loop General
                .withClosedLoopGeneral(new ClosedLoopGeneralConfigs()
                        .withContinuousWrap(true))

                // Closed Loop Ramps
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                        .withVoltageClosedLoopRampPeriod(ModuleConstants.kTurningClosedLoopRamp))
                // Motor Output
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(ModuleConstants.turningNeutralMode)
                        .withInverted(InvertedValue.CounterClockwise_Positive))

                // PID Slot0
                .withSlot0(new Slot0Configs()
                        .withKS(ModuleConstants.turningKS) 
                        .withKV(ModuleConstants.turningKV) 
                        .withKP(ModuleConstants.kPTurning) // 0.11; // An error of 1 rps results in 0.11 V output
                        .withKI(ModuleConstants.kITurning) // 0; // no output for integrated error
                        .withKD(ModuleConstants.kDTurning) // 0; // no output for error derivative
                );

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = turnMotor.getConfigurator().apply(toConfigure);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply swerve module configs, error code: " + status.toString());
        }
        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(100, turnMotor.getPosition(), turnMotor.getVelocity());
    }

    /* Initialize wheels positions */
    public void resetToAbsolute() {
        double absolutePosition = absoluteCaNcoder.getAbsolutePosition().waitForUpdate(0.25).getValueAsDouble()
                - absoluteEncoderOffset.getRotations();
        turnMotor.setPosition(absolutePosition);
    }
}
