// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.commands.shooter;

import br.megazord.frc7563.RobotState;
import br.megazord.frc7563.subsystems.shooter.ShootCalculator;
import br.megazord.frc7563.subsystems.shooter.turret.TurretSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackTargetTurretActiveCommand extends Command {
  private ShootCalculator shootCalculator = ShootCalculator.getInstance();
  private TurretSubsystem turretSubsystem;
  public TrackTargetTurretActiveCommand(TurretSubsystem turretSubsystem) {
    this.turretSubsystem = turretSubsystem;
    addRequirements(turretSubsystem);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    turretSubsystem.setTargetRotation(
      shootCalculator.getTurretAngle().minus(RobotState.getInstance().getEstimatedPose().getRotation())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    turretSubsystem.setCoastOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
