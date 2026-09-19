package br.megazord.frc7563.subsystems.shooter.turret;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.Constants.SubsystemsConstants.shooterConstants.TurretConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim turretMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.1, TurretConstants.kMotorGearRatio),
            motorModel);


    private PIDController turretPID = new PIDController(TurretConstants.kP / (2 * Math.PI), TurretConstants.kI / (2 * Math.PI), TurretConstants.kD /  (2 * Math.PI));

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public TurretIOSim() {}

    @Override
    public void updateInputs(TurretIOInputs inputs) 
    {
        if (closedLoop) {
            appliedVolts = turretPID.calculate(turretMotor.getAngularPositionRad());
        }
        else
        {
            turretPID.reset();
        }

        turretMotor.setInputVoltage(appliedVolts);
        turretMotor.update(RobotConstants.loopPeriodSecs);

        inputs.turretConnected = true;
        inputs.turretPositionRads = turretMotor.getAngularPositionRad();
        inputs.turretVelocityRadsPerSec = turretMotor.getAngularVelocityRadPerSec();
        inputs.turretAppliedVoltage = appliedVolts;
        inputs.turretSupplyCurrentAmps = turretMotor.getCurrentDrawAmps();
        inputs.turretTempCelsius = 0.0;
    }

    @Override 
    public void applyOutputs(TurretIOOutputs outputs) 
    {
        switch (outputs.mode) {
            case POSITION:
                turretPID.setSetpoint(outputs.targetRotation.getRadians());
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
