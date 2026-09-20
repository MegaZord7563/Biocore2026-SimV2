package br.megazord.frc7563.subsystems.shooter.Flywheel;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.shooter.ShooterConstants.Flywheel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(2);

    private final DCMotorSim flywheelMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.03, Flywheel.kMotorGearRatio),
            motorModel);

    private PIDController flywheelPID = new PIDController(Flywheel.kSlot0kP / (2 * Math.PI),
            Flywheel.kSlot0kI / (2 * Math.PI),
            Flywheel.kSlot0kD / (2 * Math.PI), RobotConstants.loopPeriodSecs);
    private SimpleMotorFeedforward flywheelFF = new SimpleMotorFeedforward(Flywheel.kSlot0kS / (2 * Math.PI),
            Flywheel.kSlot0kV / (2 * Math.PI), 0.0);
    private double feedForward = 0.0;

    private boolean cloosedLoop = false;
    private double appliedVolts = 0.0;

    public FlywheelIOSim() {}

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        if (cloosedLoop) {
            appliedVolts = flywheelPID.calculate(flywheelMotor.getAngularVelocityRadPerSec()) + feedForward;
        }

        flywheelMotor.update(RobotConstants.loopPeriodSecs);
        flywheelMotor.setInputVoltage(appliedVolts);

        inputs.leaderConnected = true;
        inputs.followerConnected = true;

        inputs.leaderAppliedVoltage = appliedVolts;
        inputs.leaderVelocityRadsPerSec = flywheelMotor.getAngularVelocityRadPerSec();
        inputs.leaderPositionRads = flywheelMotor.getAngularPositionRad();
        inputs.leaderSupplyCurrentAmps = flywheelMotor.getCurrentDrawAmps();
        inputs.leaderTempCelsius = 0.0;
        inputs.followerAppliedVoltage = appliedVolts;
        inputs.followerVelocityRadsPerSec = flywheelMotor.getAngularVelocityRadPerSec();
        inputs.followerPositionRads = flywheelMotor.getAngularPositionRad();
        inputs.followerSupplyCurrentAmps = flywheelMotor.getCurrentDrawAmps();
        inputs.followerTempCelsius = 0.0;
    }

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE:
                cloosedLoop = false;
                appliedVolts = outputs.voltageOut;
                break;
            case VELOCITY:
                cloosedLoop = true;
                flywheelPID.setSetpoint(outputs.velocityRadsPerSec);
                flywheelFF.calculate(outputs.velocityRadsPerSec);
                break;
            case BREAK:
                cloosedLoop = false;
                appliedVolts = 0.0;
                break;
            case COAST:
                cloosedLoop = false;
                appliedVolts = 0.0;
                break;
            case CHARACTERIZE:
                cloosedLoop = false;
                break;
            default:
                break;
        }
    }
}
