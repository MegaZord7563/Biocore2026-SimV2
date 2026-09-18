package br.megazord.frc7563.subsystems.Flywheel;

import br.megazord.frc7563.Constants.ModuleConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class FlywheelIOSim implements FlywheelIO {
    private static final DCMotor motorModel = DCMotor.getKrakenX60Foc(1);

    private final DCMotorSim leaderMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.025, ModuleConstants.kDriveMotorGearRatio),
            motorModel);

    private final DCMotorSim followerMotor = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(motorModel, 0.025, ModuleConstants.kDriveMotorGearRatio),
            motorModel);

    public FlywheelIOSim(){}

    @Override
    public void updateInputs(FlywheelIOInputs inputs){}

    @Override
    public void applyOutputs(FlywheelIOOutputs outputs){}
}
