// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter.hood;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.shooter.hood.HoodIO.HoodIOOutputs;
import br.megazord.frc7563.subsystems.shooter.hood.HoodIO.HoodIOOutputsMode;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final HoodIOOutputs outputs = new HoodIOOutputs();

  private final Alert hoodDisconnectedAlert;

  public HoodSubsystem(HoodIO io) 
  {
    this.io = io;

    hoodDisconnectedAlert = new Alert("Shooter hood motor was disconnected! Please reconnect", AlertType.kError);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if(DriverStation.isDisabled())
    {
      outputs.mode = HoodIOOutputsMode.COAST;
      io.applyOutputs(outputs);
    }

    hoodDisconnectedAlert.set(!inputs.hoodConnected && RobotConstants.enableAlerts);
  }

  public double getAngularPositionRad()
  {
    return inputs.hoodPositionRads;
  }

  public double getSupplyCurrent()
  {
    return inputs.hoodSupplyCurrentAmps;
  }

  public double getAngularVelocityRadPerSec()
  {
    return inputs.hoodVelocityRadsPerSec;
  }

  public void setCoastOut()
  {
    outputs.mode = HoodIOOutputsMode.COAST;
    io.applyOutputs(outputs);
  }

  public void setBrakeOut()
  {
    outputs.mode = HoodIOOutputsMode.BREAK;
    io.applyOutputs(outputs);
  }

  public void setTargetPositionRads(double targetPositionRads)
  {
    outputs.mode = HoodIOOutputsMode.POSITION;
    outputs.targetPositionRads = targetPositionRads;
    Logger.recordOutput("Hood/targetPosition", targetPositionRads);
    io.applyOutputs(outputs); 
  }
}
