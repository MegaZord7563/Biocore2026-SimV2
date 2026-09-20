// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.shooter.Flywheel;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.subsystems.shooter.Flywheel.FlywheelIO.FlywheelIOOutputMode;
import br.megazord.frc7563.subsystems.shooter.Flywheel.FlywheelIO.FlywheelIOOutputs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlywheelSubsystem extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Alert leaderDisconnectedAlert;
  private final Alert followerDisconnectedAlert;

  /** Creates a new FlywheelSubsystem. */
  public FlywheelSubsystem(FlywheelIO io) {
    this.io = io;

    leaderDisconnectedAlert = new Alert(
        "Disconnected leader motor on flywheel",
        AlertType.kError);
    followerDisconnectedAlert = new Alert(
        "Disconnected follower motor on flywheel",
        AlertType.kError);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    if(DriverStation.isDisabled())
    {
      outputs.mode = FlywheelIOOutputMode.COAST;
      io.applyOutputs(outputs);
    }

    followerDisconnectedAlert.set(!inputs.followerConnected);
    leaderDisconnectedAlert.set(!inputs.leaderConnected);
  }

  public void setBrakeOut()
  {
    outputs.mode = FlywheelIOOutputMode.BREAK;
    io.applyOutputs(outputs);
  }

  public void setCoastOut()
  {
    outputs.mode = FlywheelIOOutputMode.COAST;
    io.applyOutputs(outputs);
  }

  public void setVelocityModeRadsPerSec(double velocityRadsPerSec)
  {
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = velocityRadsPerSec;
    io.applyOutputs(outputs);

    Logger.recordOutput("Flywheel/velocitySetpointRadsPerSec", velocityRadsPerSec);
  }

  public double getFlywheelVelocityRadsPerSec()
  {
    return inputs.leaderVelocityRadsPerSec;
  }

  public double getFlywheelSupplyCurrentAmps()
  {
    return inputs.leaderSupplyCurrentAmps;
  }

  public double getFlywheelAppliedVolts()
  {
    return inputs.leaderAppliedVoltage;
  }
}
