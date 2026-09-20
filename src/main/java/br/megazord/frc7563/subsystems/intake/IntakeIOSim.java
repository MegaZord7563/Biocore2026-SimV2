// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.intake;

import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO 
{
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim pivotMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.025, 1),// ModuleConstants.kDriveMotorGearRatio),
            motorModel);

    private final DCMotorSim rollersMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.004, 1),// ModuleConstants.kTurningMotorGearRatio),
            motorModel);

    private final PIDController pivotPID = new PIDController(0.0, 0.0, 0.0);
    private final PIDController rollersPID = new PIDController(0.0,0.0,0,0);
    

    private double pivotAppliedVolts = 0.0;
    private double rollersAppliedVolts = 0.0;
    private boolean pivotClosedLoop = false;
    private boolean rollersClosedLoop = false;

    public IntakeIOSim() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        if(pivotClosedLoop)
        {  
            pivotAppliedVolts = pivotPID.calculate(pivotMotor.getAngularPositionRad());
        } else 
        {
            pivotPID.reset();
        }

        if(rollersClosedLoop)
        {  
            rollersAppliedVolts = rollersPID.calculate(rollersMotor.getAngularVelocityRadPerSec());
        } else 
        {
            rollersPID.reset();
        }

        pivotMotor.setInputVoltage(pivotAppliedVolts);
        rollersMotor.setInputVoltage(rollersAppliedVolts);
        pivotMotor.update(RobotConstants.loopPeriodSecs);
        rollersMotor.update(RobotConstants.loopPeriodSecs);

        inputs.pivotConnected = true;
        inputs.pivotPositionRads = pivotMotor.getAngularPositionRad();
        inputs.pivotVelocityRadsPerSec = pivotMotor.getAngularVelocityRadPerSec();
        inputs.pivotAppliedVolts = pivotAppliedVolts;
        inputs.pivotSupplyCurrentAmps = pivotMotor.getCurrentDrawAmps();
        inputs.pivotTempCelsius = 0.0;

        inputs.rollersConnected = true;
        inputs.rollerPositionRads = rollersMotor.getAngularPositionRad();
        inputs.rollersVelocityRadsPerSec = rollersMotor.getAngularVelocityRadPerSec();
        inputs.rollersAppliedVolts = rollersAppliedVolts;
        inputs.rollersSupplyCurrentAmps = rollersMotor.getCurrentDrawAmps();
        inputs.rollersTempCelsius = 0.0;
    }

    @Override
    public void applyOutputs(IntakeIOOutputs outputs) 
    {
        switch (outputs.mode) {
            case COAST, BREAK:
                rollersClosedLoop = false;
                pivotClosedLoop = false;
                rollersAppliedVolts = 0.0;
                pivotAppliedVolts = 0.0;
                break;
            case RUN:
                rollersClosedLoop = true;
                pivotClosedLoop = false;
                rollersPID.setSetpoint(outputs.rollersSpeedRadPerSec);
                pivotAppliedVolts = 0.0;
                break;
            case MOVE:
                rollersClosedLoop = true;
                pivotClosedLoop = true;
                rollersPID.setSetpoint(outputs.rollersSpeedRadPerSec);
                pivotPID.setSetpoint(outputs.pivotTargetPositionRads);
                break;
            case CHARACTERIZE:
                rollersClosedLoop = false;
                pivotClosedLoop = true;
                rollersAppliedVolts = outputs.rollersCharacterizationOutput;
                pivotPID.setSetpoint(outputs.pivotTargetPositionRads);
                break;
            default:
                break;
        }
    }
}
