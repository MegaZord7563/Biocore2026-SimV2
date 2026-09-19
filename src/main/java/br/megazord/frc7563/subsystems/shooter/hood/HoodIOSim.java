package br.megazord.frc7563.subsystems.shooter.hood;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.Constants.SubsystemsConstants.shooterConstants.HoodConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HoodIOSim implements HoodIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX44Foc(1);

    private final DCMotorSim hoodMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.1, HoodConstants.kMotorGearRatio),
            motorModel);


    private PIDController hoodPID = new PIDController(HoodConstants.kP / (2 * Math.PI), HoodConstants.kI / (2 * Math.PI), HoodConstants.kD /  (2 * Math.PI));

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public HoodIOSim() {}

    @Override
    public void updateInputs(HoodIOInputs inputs) 
    {
        if (closedLoop) {
            appliedVolts = hoodPID.calculate(hoodMotor.getAngularPositionRad());
        }
        else
        {
            hoodPID.reset();
        }

        hoodMotor.setInputVoltage(appliedVolts);
        hoodMotor.update(RobotConstants.loopPeriodSecs);

        inputs.hoodConnected = true;
        inputs.hoodPositionRads = hoodMotor.getAngularPositionRad();
        inputs.hoodVelocityRadsPerSec = hoodMotor.getAngularVelocityRadPerSec();
        inputs.hoodAppliedVoltage = appliedVolts;
        inputs.hoodSupplyCurrentAmps = hoodMotor.getCurrentDrawAmps();
        inputs.hoodTempCelsius = 0.0;
    }

    @Override 
    public void applyOutputs(HoodIOOutputs outputs) 
    {
        switch (outputs.mode) {
            case POSITION:
                hoodPID.setSetpoint(outputs.targetPositionRads);
                closedLoop = true;
                break;

            case VOLTAGE:
                appliedVolts = outputs.voltageOut;
                closedLoop = false;
                break;

            case BREAK:
                appliedVolts = 0.0;
                closedLoop = false;
                break;

            case COAST:
                appliedVolts = 0.0;
                closedLoop = false;
                break;
        
            default:
                break;
        }
    }
}
