// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.feeder;

public class FeederIOTalonFX implements FeederIO {

    public FeederIOTalonFX() {
        initializeMotors();
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {}

    @Override
    public void applyOutputs(FeederIOOutputs outputs) {}

    private void initializeMotors() {}
}