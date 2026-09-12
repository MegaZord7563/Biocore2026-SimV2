// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import java.util.function.Supplier;

import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIOSim implements GyroIO
{
    private Supplier<Rotation2d> angularVelocity;
    private Rotation2d yaw = Rotation2d.kZero;
    public GyroIOSim(Supplier<Rotation2d> angularVelocity) 
    {
        this.angularVelocity = angularVelocity;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs)
    {
        yaw = yaw.plus(angularVelocity.get().times(RobotConstants.loopPeriodSecs));
        inputs.robotRotation2d = yaw;
    }

    @Override
    public void setPosition(Rotation2d angle)
    {
        yaw  = angle;
    }

    @Override
    public void resetPosition()
    {
        yaw = Rotation2d.kZero;
    }
}
