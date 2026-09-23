// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.subsystems.feeder.FeederIO.FeederIOOutputMode;
import br.megazord.frc7563.subsystems.feeder.FeederIO.FeederIOOutputs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();
  private final FeederIOOutputs outputs = new FeederIOOutputs();

  private final Alert leaderDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  /** Creates a new FlywheelSubsystem. */
  public FeederSubsystem(FeederIO io) {
    this.io = io;

    leaderDisconnectedAlert = new Alert(
        "Disconnected leader motor on Feeder",
        AlertType.kError);
    followerDisconnectedAlert = new Alert(
        "Disconnected follower motor on Feeder",
        AlertType.kError);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    if(DriverStation.isDisabled())
    {
      outputs.mode = FeederIOOutputMode.COAST;
      io.applyOutputs(outputs);
    }

    followerDisconnectedAlert.set(!inputs.followerConnected);
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
  }

  public void setBrakeOut()
  {
    outputs.mode = FeederIOOutputMode.BREAK;
    io.applyOutputs(outputs);
  }

  public void setCoastOut()
  {
    outputs.mode = FeederIOOutputMode.COAST;
    io.applyOutputs(outputs);
  }

  public void setVelocityModeRadsPerSec(double velocityRadsPerSec)
  {
    outputs.mode = FeederIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    io.applyOutputs(outputs);

    Logger.recordOutput("Flywheel/velocitySetpointRadsPerSec", velocityRadsPerSec);
  }

  public void setFeederVoltageOut(double voltageOut)
  {
    outputs.mode = FeederIOOutputMode.VOLTAGE;
    outputs.voltageOut = voltageOut;
    io.applyOutputs(outputs);
  }

  public double getFeederVelocityRadsPerSecLeader()
  {
    return inputs.leaderVelocityRadsPerSec;
  }

  public double getFeederSupplyCurrentAmpsLeader()
  {
    return inputs.leaderSupplyCurrentAmps;
  }

  public double getFeederAppliedVoltsLeader()
  {
    return inputs.leaderAppliedVoltage;
  }
}
