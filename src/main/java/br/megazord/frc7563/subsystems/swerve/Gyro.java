// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import br.megazord.frc7563.Constants.RobotConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;

/** 
 * This class is a placeholder for the Gyro implementation. It can be extended to include methods for interacting with a gyroscope sensor, such as getting the current angle, resetting the gyro, and updating gyro inputs.
 */
public class Gyro 
{
    private GyroIO io;
    private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    private Alert gyroDisconnectedAlert = new Alert("Gyro Disconnected Alert", edu.wpi.first.wpilibj.Alert.AlertType.kError);


    public Gyro(GyroIO io) 
    {
        this.io = io;
    }

    public void periodic() 
    {
        io.updateInputs(inputs);

        gyroDisconnectedAlert.set(!isConnected() && RobotConstants.enableAlerts);
    }

    public void resetPosition() 
    {
        io.resetPosition();
    }

    public void setPosition(Rotation2d angle) 
    {
        io.setPosition(angle);
    }

    public boolean isConnected()
    {
        return inputs.connected;
    }

    public Rotation2d getAngle() 
    {
        return inputs.robotRotation2d;
    }
}
