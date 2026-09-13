// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2FeaturesConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;

import br.megazord.frc7563.Constants.DriveConstants;
import br.megazord.frc7563.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class GyroIOPygeon2 implements GyroIO {
    CANBus swerveCAN = ModuleConstants.swerveCAN;
    // Create Pigeon2 gyro
    private Pigeon2 gyro = new Pigeon2(DriveConstants.kPigeonPort, swerveCAN);

    public GyroIOPygeon2() 
    {
        initializePigeon2();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) 
    {
        inputs.connected = gyro.isConnected();
        inputs.robotRotation2d = Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    @Override
    public void setPosition(Rotation2d angle) 
    {
        gyro.setYaw(angle.getDegrees());
    }

    @Override
    public void resetPosition() 
    {
        gyro.setYaw(0.0);
    }

    /**
     * Initialize Pigeon2 device from the configurator object
     * 
     * @link https://ctre.download/files/user-manual/Pigeon2%20User's%20Guide.pdf
     * @param cfg Configurator of the Pigeon2 device
     */
    private void initializePigeon2() {
        // Create a new Pigeon2Configurator object
        Pigeon2Configuration configs = new Pigeon2Configuration();
        // Clear the sticky faults
        gyro.clearStickyFaults();
        // Set the yaw to 0
        // gyro.setYaw(0, 0);

        // Apply the configuration to the Pigeon2 device
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        // Set the yaw to 0
        // gyro.getConfigurator().setYaw(0, 0);

        configs.withGyroTrim(new GyroTrimConfigs()
                .withGyroScalarX(-4.5 - 3.2) // -2.9779;
                .withGyroScalarY(-4.5 - 3.2)
                .withGyroScalarZ(-4.5 - 3.2));
        // We want the thermal comp and no-motion cal enabled, with the compass disabled
        // for best behavior

        configs.withPigeon2Features(new Pigeon2FeaturesConfigs()
                .withDisableNoMotionCalibration(false)
                .withDisableTemperatureCompensation(false)
                .withEnableCompass(false));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = gyro.getConfigurator().apply(configs);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply gyro configs, error code: " + status.toString());
        }

        /* And initialize yaw to 0 */
        // MUST NOT TOUCH

        /* Speed up signals to an appropriate rate */
        BaseStatusSignal.setUpdateFrequencyForAll(
                100, gyro.getYaw(),
                gyro.getGravityVectorZ(),
                gyro.getAccelerationZ());

        gyro.setYaw(0, 0);
    }
}
