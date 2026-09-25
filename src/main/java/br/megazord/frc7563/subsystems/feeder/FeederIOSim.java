// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.feeder;

import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulated IO for the feeder. Leader + follower are aligned, so they are modeled as one motor pair spinning together. */
public class FeederIOSim implements FeederIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(2);

    private final DCMotorSim feederMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.001, FeederConstants.kMotorGearRatio),
            motorModel);

    private final PIDController feederPID = new PIDController(
            FeederConstants.kSlot0kP / (2 * Math.PI),
            FeederConstants.kSlot0kI / (2 * Math.PI),
            FeederConstants.kSlot0kD / (2 * Math.PI),
            RobotConstants.loopPeriodSecs);
    private final SimpleMotorFeedforward feederFF = new SimpleMotorFeedforward(
            FeederConstants.kSlot0kS / (2 * Math.PI),
            FeederConstants.kSlot0kV / (2 * Math.PI), 0.0);

    private boolean closedLoop = false;
    private double feedForward = 0.0;
    private double appliedVolts = 0.0;

    public FeederIOSim() {}

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = feederPID.calculate(feederMotor.getAngularVelocityRadPerSec()) + feedForward;
        }

        feederMotor.setInputVoltage(appliedVolts);
        feederMotor.update(RobotConstants.loopPeriodSecs);

        inputs.leaderConnected = true;
        inputs.leaderPositionRads = feederMotor.getAngularPositionRad();
        inputs.leaderVelocityRadsPerSec = feederMotor.getAngularVelocityRadPerSec();
        inputs.leaderAppliedVoltage = appliedVolts;
        inputs.leaderSupplyVoltage = 12.0;
        inputs.leaderSupplyCurrentAmps = feederMotor.getCurrentDrawAmps();
        inputs.leaderTempCelsius = 0.0;

        // follower is aligned to the leader, so it mirrors the same motion
        inputs.followerConnected = true;
        inputs.followerPositionRads = feederMotor.getAngularPositionRad();
        inputs.followerVelocityRadsPerSec = feederMotor.getAngularVelocityRadPerSec();
        inputs.followerAppliedVoltage = appliedVolts;
        inputs.followerSupplyCurrentAmps = feederMotor.getCurrentDrawAmps();
        inputs.followerTempCelsius = 0.0;
    }

    @Override
    public void applyOutputs(FeederIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE:
                closedLoop = false;
                appliedVolts = outputs.voltageOut;
                break;
            case VELOCITY:
                closedLoop = true;
                feederPID.setSetpoint(outputs.velocityRadsPerSec);
                feedForward = feederFF.calculate(outputs.velocityRadsPerSec);
                break;
            case BREAK:
            case COAST:
                closedLoop = false;
                appliedVolts = 0.0;
                break;
            case CHARACTERIZE:
                closedLoop = false;
                break;
            default:
                break;
        }
    }
}
