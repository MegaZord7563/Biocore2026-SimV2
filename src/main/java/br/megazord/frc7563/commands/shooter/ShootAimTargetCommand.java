// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.commands.shooter;

import br.megazord.frc7563.RobotState;
import br.megazord.frc7563.Constants.DriveConstants.DriveMode;
import br.megazord.frc7563.subsystems.shooter.ShootCalculator;
import br.megazord.frc7563.subsystems.shooter.Flywheel.FlywheelSubsystem;
import br.megazord.frc7563.subsystems.shooter.hood.HoodSubsystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAimTargetCommand extends Command {
  private ShootCalculator shootCalculator = ShootCalculator.getInstance();
  private HoodSubsystem hoodSubsystem;
  private FlywheelSubsystem flywheelSubsystem;

  public ShootAimTargetCommand(HoodSubsystem hoodSubsystem, FlywheelSubsystem flywheelSubsystem) {
    this.hoodSubsystem = hoodSubsystem;
    this.flywheelSubsystem = flywheelSubsystem;

    addRequirements(hoodSubsystem, flywheelSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    RobotState.getInstance().setSwerveDriveMode(DriveMode.SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    flywheelSubsystem.setVelocityModeRadsPerSec(Units.rotationsToRadians(shootCalculator.getFlywheelSpeed()));
    hoodSubsystem.setTargetPositionRads(Units.degreesToRadians(shootCalculator.getHoodAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    hoodSubsystem.setTargetPositionRads(0.0);
    flywheelSubsystem.setFlywheelVoltageOut(7);
    RobotState.getInstance().setSwerveDriveMode(DriveMode.FAST);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotState.getInstance().isNearTrench();
  }
}
