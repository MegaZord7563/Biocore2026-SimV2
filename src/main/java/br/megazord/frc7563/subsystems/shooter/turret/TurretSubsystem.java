// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import br.megazord.frc7563.subsystems.shooter.turret.TurretIO.TurretIOOutputsMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  private final Alert turretDisconnectedAlert;

  public TurretSubsystem(TurretIO io) 
  {
    this.io = io;

    turretDisconnectedAlert = new Alert("Shooter Turret motor was disconnected! Please reconnect", AlertType.kError);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if(DriverStation.isDisabled())
    {
      outputs.mode = TurretIOOutputsMode.COAST;
      io.applyOutputs(outputs);
    }

    turretDisconnectedAlert.set(!inputs.turretConnected && RobotConstants.enableAlerts);
  }

  public double getAngularPositionRad()
  {
    return inputs.turretPositionRads;
  }

  public double getSupplyCurrent()
  {
    return inputs.turretSupplyCurrentAmps;
  }

  public double getAngularVelocityRadPerSec()
  {
    return inputs.turretVelocityRadsPerSec;
  }

  public void setCoastOut()
  {
    outputs.mode = TurretIOOutputsMode.COAST;
    io.applyOutputs(outputs);
  }

  public void setBrakeOut()
  {
    outputs.mode = TurretIOOutputsMode.BREAK;
    io.applyOutputs(outputs);
  }

  public void setTargetRotation(Rotation2d targetRotation)
  {
    outputs.mode = TurretIOOutputsMode.POSITION;
    outputs.targetRotation = targetRotation;
    Logger.recordOutput("Turret/targetRotation", targetRotation);
    io.applyOutputs(outputs); 
  }
}
