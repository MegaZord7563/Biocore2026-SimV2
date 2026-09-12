// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO 
{
    @AutoLog
    public class GyroIOInputs 
    {
        boolean connected = false;
        Rotation2d robotRotation2d = new Rotation2d();
    }

    public default void resetPosition(){}

    public default void setPosition(Rotation2d angle){}
    
    public default void updateInputs(GyroIOInputs inputs){}
}