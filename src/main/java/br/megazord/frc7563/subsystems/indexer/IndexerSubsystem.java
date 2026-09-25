// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.subsystems.indexer.IndexerIO.IndexerIOOutputMode;
import br.megazord.frc7563.subsystems.indexer.IndexerIO.IndexerIOOutputs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
  private final IndexerIOOutputs outputs = new IndexerIOOutputs();

  private final Alert indexerDisconnectedAlert;

  public IndexerSubsystem(IndexerIO io) {
    this.io = io;

    indexerDisconnectedAlert = new Alert(
        "Disconnected motor on Indexer",
        AlertType.kError);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);

    if (DriverStation.isDisabled()) 
    {
      outputs.mode = IndexerIOOutputMode.BREAK;
      io.applyOutputs(outputs);
    }

    indexerDisconnectedAlert.set(!inputs.indexerConnected);
  }

  public void setBrakeOut() 
  {
    outputs.mode = IndexerIOOutputMode.BREAK;
    io.applyOutputs(outputs);
  }

  public void setCoastOut() 
  {
    outputs.mode = IndexerIOOutputMode.COAST;
    io.applyOutputs(outputs);
  }

  public void setVelocityModeRadsPerSec(double velocityRadsPerSec) 
  {
    outputs.mode = IndexerIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    io.applyOutputs(outputs);

    Logger.recordOutput("Indexer/velocitySetpointRadsPerSec", velocityRadsPerSec);
  }

  public void setIndexerVoltageOut(double voltageOut) 
  {
    outputs.mode = IndexerIOOutputMode.VOLTAGE;
    outputs.voltageOut = voltageOut;
    io.applyOutputs(outputs);
  }

  public void runIndexer() 
  {
    setVelocityModeRadsPerSec(IndexerConstants.kIndexerRunRadsPerSec);
  }

  public void stop() 
  {
    setBrakeOut();
  }

  public double getIndexerVelocityRadsPerSec() 
  {
    return inputs.indexerVelocityRadsPerSec;
  }

  public double getIndexerSupplyCurrentAmps() 
  {
    return inputs.indexerSupplyCurrentAmps;
  }

  public double getIndexerAppliedVolts() 
  {
    return inputs.indexerAppliedVoltage;
  }
}
