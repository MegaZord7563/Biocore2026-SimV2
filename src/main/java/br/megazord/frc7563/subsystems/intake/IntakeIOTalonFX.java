// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package br.megazord.frc7563.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import br.megazord.frc7563.subsystems.intake.IntakeConstants.Pivot;
import br.megazord.frc7563.subsystems.intake.IntakeConstants.Rollers;
import edu.wpi.first.math.util.Units;

/** Simulated IO for the intake (pivot + rollers). */
public class IntakeIOTalonFX implements IntakeIO 
{
    private final TalonFX pivotMotor = new TalonFX(Pivot.kMotorId, CANBus.roboRIO());
    private final TalonFX rollersMotor = new TalonFX(Rollers.kMotorId, CANBus.roboRIO());

    // create a velocity closed-loop request, voltage output, slot 0 configs
    private final VelocityVoltage rollersPID = new VelocityVoltage(0).withSlot(0);
    // create a position closed-loop request, voltage output, slot 0 configs
    private final PositionVoltage pivotPID = new PositionVoltage(0).withSlot(0);
    /* Keep a brake request so we can disable the motor */
    private final NeutralOut breakOut = new NeutralOut();
    /* Keep a coast request so we can disable the motor */
    private final CoastOut coastOut = new CoastOut();

    public IntakeIOTalonFX() {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) 
    {
        inputs.pivotConnected = pivotMotor.isConnected();
        inputs.pivotPositionRads = Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble());
        inputs.pivotVelocityRadsPerSec = Units.rotationsToRadians(pivotMotor.getVelocity().getValueAsDouble());
        inputs.pivotAppliedVolts = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotSupplyCurrentAmps = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.pivotTempCelsius = pivotMotor.getDeviceTemp().getValueAsDouble();

        inputs.rollersConnected = rollersMotor.isConnected();
        inputs.rollerPositionRads = Units.rotationsToRadians(rollersMotor.getPosition().getValueAsDouble());
        inputs.rollersVelocityRadsPerSec = Units.rotationsToRadians(rollersMotor.getVelocity().getValueAsDouble());
        inputs.rollersAppliedVolts = rollersMotor.getMotorVoltage().getValueAsDouble();
        inputs.rollersSupplyCurrentAmps = rollersMotor.getSupplyCurrent().getValueAsDouble();
        inputs.rollersTempCelsius = rollersMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void applyOutputs(IntakeIOOutputs outputs) 
    {
        switch (outputs.mode) {
            case COAST:
                pivotMotor.setControl(coastOut);
                rollersMotor.setControl(coastOut);
                break;
            case BREAK:
                pivotMotor.setControl(breakOut);
                rollersMotor.setControl(breakOut);
                break;
            case RUN:
                rollersMotor.setControl(rollersPID.withVelocity(Units.radiansToRotations(outputs.rollersSpeedRadPerSec)));
                pivotMotor.setControl(coastOut);
                break;
            case MOVE:
                rollersMotor.setControl(rollersPID.withVelocity(Units.radiansToRotations(outputs.rollersSpeedRadPerSec)));
                pivotMotor.setControl(pivotPID.withPosition(Units.radiansToRotations(outputs.pivotTargetPositionRads)));
                break;
            case CHARACTERIZE:
                // Not implemented yet. 
                pivotMotor.setControl(coastOut);
                rollersMotor.setControl(coastOut);
                break;
            default:
                break;
        }
    }
}
