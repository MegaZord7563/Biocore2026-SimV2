// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

// local imports
import br.megazord.frc7563.Constants.ModuleConstants;
import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
// wpilib imports
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class SwerveModuleIOSim implements SwerveModuleIO {

    private static final DCMotor driveMotorModel = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor turnMotorModel = DCMotor.getKrakenX44Foc(1);

    private final DCMotorSim driveMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorModel, 0.025, ModuleConstants.kDriveMotorGearRatio),
            driveMotorModel);

    private final DCMotorSim turnMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel, 0.004, ModuleConstants.kTurningMotorGearRatio),
            turnMotorModel);

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;

    private PIDController drivePID = new PIDController(ModuleConstants.kPdriving, ModuleConstants.kIdriving,
            ModuleConstants.kDdriving, RobotConstants.loopPeriodSecs);
    private SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(ModuleConstants.driveKS / (2 * Math.PI), ModuleConstants.driveKV / (2 * Math.PI), ModuleConstants.driveKA / (2 * Math.PI));
    private double feedForward = 0.0;
    
    private PIDController turnPID = new PIDController(ModuleConstants.kPTurning / (2 * Math.PI), ModuleConstants.kITurning / (2 * Math.PI),
            ModuleConstants.kDTurning / (2 * Math.PI), RobotConstants.loopPeriodSecs);

    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleIOSim() {
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = drivePID.calculate(driveMotor.getAngularVelocityRadPerSec()) + feedForward;
        } else {
            drivePID.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnPID.calculate(turnMotor.getAngularPositionRad());
        } else {
            turnPID.reset();
        }

        // Update simulation state
        driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveMotor.update(RobotConstants.loopPeriodSecs);
        turnMotor.update(RobotConstants.loopPeriodSecs);

        inputs.driveConnected = true;
        inputs.drivePositionRads = driveMotor.getAngularPositionRad();
        inputs.driveVelocityRadsPerSec = driveMotor.getAngularVelocityRadPerSec();
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveSupplyCurrentAmps = Math.abs(driveMotor.getCurrentDrawAmps());
        inputs.driveTempCelsius = 0.0;

        inputs.turnConnected = true;
        inputs.turnPositionRads = new Rotation2d(turnMotor.getAngularPositionRad());
        inputs.turnAbsolutePositionRads = new Rotation2d(turnMotor.getAngularPositionRad());
        inputs.turnTempCelsius = 0.0;
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnSupplyCurrentAmps = Math.abs(turnMotor.getCurrentDrawAmps());
        inputs.turnVelocityRadsPerSec = turnMotor.getAngularVelocityRadPerSec();
    }

    @Override
    public void applyOutputs(SwerveModuleIOOutputs outputs) {
        switch (outputs.mode) {
            case COAST, BREAK:
                driveClosedLoop = false;
                turnClosedLoop = false;
                driveAppliedVolts = 0.0;
                turnAppliedVolts = 0.0;
                break;

            case DRIVE:
                driveClosedLoop = true;
                turnClosedLoop = true;
                drivePID.setSetpoint(outputs.driveVelocityRadPerSec);
                feedForward = driveFF.calculate(outputs.driveVelocityRadPerSec);
                turnPID.setSetpoint(outputs.turnRotation.getRadians());
                break;

            case CHARACTERIZE:
                driveClosedLoop = false;
                turnClosedLoop = true;
                turnPID.setSetpoint(outputs.turnRotation.getRadians());
      }
    }

    @Override
    public void resetDriveEncoders() {
        driveMotor.setAngle(0.0);
    }
}
