package br.megazord.frc7563.subsystems.shooter.turret;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Turret;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim turretMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.5, Turret.kMotorGearRatio),
            motorModel);

    private final Constraints constraints =
    new TrapezoidProfile.Constraints(
        Units.rotationsToRadians(12.0),
        Units.rotationsToRadians(6.0)
    );

    private ProfiledPIDController turretPID = new ProfiledPIDController(
        Turret.kP / (2 * Math.PI), 
        Turret.kI / (2 * Math.PI),
        Turret.kD / (2 * Math.PI), 
        constraints);

    private boolean closedLoop = false;
    private double appliedVolts = 0.0;

    public TurretIOSim() {
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = turretPID.calculate(turretMotor.getAngularPositionRad());
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
    public void applyOutputs(TurretIOOutputs outputs) {
        switch (outputs.mode) {
            case POSITION:
                turretPID.setGoal(outputs.targetRotation.getRadians());
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
