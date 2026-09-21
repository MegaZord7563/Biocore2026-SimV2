// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.intake.IntakeConstants.Pivot;
import br.megazord.frc7563.subsystems.intake.IntakeConstants.Rollers;
import br.megazord.frc7563.subsystems.intake.IntakeIO.IntakeIOOutputMode;
import br.megazord.frc7563.subsystems.intake.IntakeIO.IntakeIOOutputs;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIOOutputs outputs = new IntakeIOOutputs();

  private final Alert pivotDisconnectedAlert;
  private final Alert rollersDisconnectedAlert;

  public IntakeSubsystem(IntakeIO io) 
  {
    this.io = io;

    pivotDisconnectedAlert = new Alert("Intake pivot motor was disconnected! Please reconnect", AlertType.kError);
    rollersDisconnectedAlert = new Alert("Intake rollers motor was disconnected! Please reconnect", AlertType.kError);
  }

  @Override
  public void periodic() 
  {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    if(DriverStation.isDisabled())
    {
      outputs.mode = IntakeIOOutputMode.COAST;
      io.applyOutputs(outputs);
    }

    pivotDisconnectedAlert.set(!inputs.pivotConnected && RobotConstants.enableAlerts);
    rollersDisconnectedAlert.set(!inputs.rollersConnected && RobotConstants.enableAlerts);
  }

  public void setCoastOut()
  {
    outputs.mode = IntakeIOOutputMode.COAST;
    io.applyOutputs(outputs);
  }

  public void setBrakeOut()
  {
    outputs.mode = IntakeIOOutputMode.BREAK;
    io.applyOutputs(outputs);
  }

  /** Spins the rollers only; the pivot is left uncontrolled (coasts). */
  public void runRollers(double rollersSpeedRadPerSec)
  {
    outputs.mode = IntakeIOOutputMode.RUN;
    outputs.rollersSpeedRadPerSec = rollersSpeedRadPerSec;
    io.applyOutputs(outputs);

    Logger.recordOutput("Intake/rollersSetpointRadPerSec", rollersSpeedRadPerSec);
  }

  /** Drives the pivot to a target position while also running the rollers at the given speed. */
  public void moveTo(double pivotTargetPositionRads, double rollersSpeedRadPerSec)
  {
    outputs.mode = IntakeIOOutputMode.MOVE;
    outputs.pivotTargetPositionRads = pivotTargetPositionRads;
    outputs.rollersSpeedRadPerSec = rollersSpeedRadPerSec;
    io.applyOutputs(outputs);

    Logger.recordOutput("Intake/pivotSetpointRads", pivotTargetPositionRads);
    Logger.recordOutput("Intake/rollersSetpointRadPerSec", rollersSpeedRadPerSec);
  }

  /** Drives the pivot to a target position while also running the rollers at the given speed. */
  public void moveTo(double pivotTargetPositionRads)
  {
    outputs.mode = IntakeIOOutputMode.MOVE;
    outputs.pivotTargetPositionRads = pivotTargetPositionRads;
    outputs.rollersSpeedRadPerSec = 0.0;
    io.applyOutputs(outputs);

    Logger.recordOutput("Intake/pivotSetpointRads", pivotTargetPositionRads);
    Logger.recordOutput("Intake/rollersSetpointRadPerSec", 0.0);
  }

  /** Deploys the intake and spins the rollers inward to pick up a game piece. */
  public void intake()
  {
    moveTo(Pivot.kDeployedPositionRads, Rollers.kIntakeSpeedRadsPerSec);
  }

  /** Deploys the intake and spins the rollers outward to eject a game piece. */
  public void outtake()
  {
    moveTo(Pivot.kDeployedPositionRads, Rollers.kOuttakeSpeedRadsPerSec);
  }

  /** Retracts the intake and stops the rollers. */
  public void stow()
  {
    moveTo(Pivot.kStowedPositionRads, 0.0);
  }

  public double getPivotPositionRads()
  {
    return inputs.pivotPositionRads;
  }

  public double getPivotVelocityRadsPerSec()
  {
    return inputs.pivotVelocityRadsPerSec;
  }

  public double getRollersVelocityRadsPerSec()
  {
    return inputs.rollersVelocityRadsPerSec;
  }

  public double getPivotSupplyCurrentAmps()
  {
    return inputs.pivotSupplyCurrentAmps;
  }

  public double getRollersSupplyCurrentAmps()
  {
    return inputs.rollersSupplyCurrentAmps;
  }

  /** True once the pivot is within tolerance of the last commanded target. */
  public boolean atPivotSetpoint(double toleranceRads)
  {
    return Math.abs(inputs.pivotPositionRads - outputs.pivotTargetPositionRads) < toleranceRads;
  }
}
