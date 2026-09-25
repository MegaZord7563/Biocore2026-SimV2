// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.indexer;

import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulated IO for the indexer mechanism using WPILib DCMotorSim, PIDController and feedforward. */
public class IndexerIOSim implements IndexerIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim indexerMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.001, IndexerConstants.kMotorGearRatio),
            motorModel);

    private final PIDController indexerPID = new PIDController(
            IndexerConstants.kSlot0kP / (2 * Math.PI),
            IndexerConstants.kSlot0kI / (2 * Math.PI),
            IndexerConstants.kSlot0kD / (2 * Math.PI),
            RobotConstants.loopPeriodSecs);

    private final SimpleMotorFeedforward indexerFF = new SimpleMotorFeedforward(
            IndexerConstants.kSlot0kS / (2 * Math.PI),
            IndexerConstants.kSlot0kV / (2 * Math.PI), 0.0);

    private boolean closedLoop = false;
    private double feedForward = 0.0;
    private double appliedVolts = 0.0;

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        if (closedLoop) {
            appliedVolts = indexerPID.calculate(indexerMotor.getAngularVelocityRadPerSec()) + feedForward;
        }

        indexerMotor.setInputVoltage(appliedVolts);
        indexerMotor.update(RobotConstants.loopPeriodSecs);

        inputs.indexerConnected = true;
        inputs.indexerPositionRads = indexerMotor.getAngularPositionRad();
        inputs.indexerVelocityRadsPerSec = indexerMotor.getAngularVelocityRadPerSec();
        inputs.indexerAppliedVoltage = appliedVolts;
        inputs.indexerSupplyVoltage = 12.0;
        inputs.indexerSupplyCurrentAmps = indexerMotor.getCurrentDrawAmps();
        inputs.indexerTempCelsius = 0.0;
    }

    @Override
    public void applyOutputs(IndexerIOOutputs outputs) {
        switch (outputs.mode) {
            case VOLTAGE:
                closedLoop = false;
                appliedVolts = outputs.voltageOut;
                break;
            case VELOCITY:
                closedLoop = true;
                indexerPID.setSetpoint(outputs.velocityRadsPerSec);
                feedForward = indexerFF.calculate(outputs.velocityRadsPerSec);
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
