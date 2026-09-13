package br.megazord.frc7563.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import br.megazord.frc7563.Constants.RobotConstants;
import br.megazord.frc7563.subsystems.swerve.SwerveModuleIO.SwerveModuleIOOutputMode;
import br.megazord.frc7563.subsystems.swerve.SwerveModuleIO.SwerveModuleIOOutputs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();
    private final SwerveModuleIOOutputs outputs = new SwerveModuleIOOutputs();
    private final String position;

    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert encoderDisconnectedAlert;

    private SwerveModuleState desiredState = new SwerveModuleState();

    public SwerveModule(SwerveModuleIO io, String position) {
        this.io = io;
        this.position = position;
        driveDisconnectedAlert = new Alert(
                "Disconnected drive motor on module " + position,
                AlertType.kError);
        turnDisconnectedAlert = new Alert(
                "Disconnected turn motor on module " + position,
                AlertType.kError);
        encoderDisconnectedAlert = new Alert(
                "Disconnected encoder on module " + position,
                AlertType.kError);
    }

    public void periodic() {
        io.updateInputs(inputs);
        io.applyOutputs(outputs);
        Logger.processInputs("SwerveDrive/Module" + position, inputs);

        driveDisconnectedAlert.set(!inputs.driveConnected && RobotConstants.enableAlerts);
        turnDisconnectedAlert.set(!inputs.turnConnected && RobotConstants.enableAlerts);
        encoderDisconnectedAlert.set(!inputs.cancoderConnected && RobotConstants.enableAlerts);
    }

    /**
     * Sets the desired state for the module.
     * 
     * @param state Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState state) {
        desiredState = state;

        SwerveModuleState correctState = new SwerveModuleState(state.speedMetersPerSecond, state.angle);

        correctState.optimize(inputs.turnPositionRads);
        correctState.cosineScale(inputs.turnPositionRads);
        correctState.angle = correctState.angle.plus(Rotation2d.fromRadians(inputs.chassisAngularOffset));

        outputs.mode = SwerveModuleIOOutputMode.DRIVE;
        double speedRotationsPerSecond = SwerveConversions
                .metersPerSecToWheelRotationsPerSec(correctState.speedMetersPerSecond);
        outputs.driveVelocityRadPerSec = Units.rotationsToRadians(speedRotationsPerSecond);
        outputs.turnRotation = correctState.angle;
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        outputs.mode = SwerveModuleIOOutputMode.CHARACTERIZE;
        outputs.driveCharacterizationOutput = output;
        outputs.turnRotation = Rotation2d.kZero;
    }

    /** Disables all motor outputs in brake mode. */
    public void brake() {
        outputs.mode = SwerveModuleIOOutputMode.BRAKE;
        outputs.driveVelocityRadPerSec = 0.0;
    }

    /** Disables all motor outputs in coast mode. */
    public void coast() {
        outputs.mode = SwerveModuleIOOutputMode.COAST;
        outputs.driveVelocityRadPerSec = 0.0;
    }

    /** Returns whether the motors are connected. */
    public boolean isConnected() {
        return inputs.driveConnected && inputs.turnConnected;
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turnPositionRads;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return SwerveConversions.wheelRotationsToMeters(Units.radiansToRotations(inputs.drivePositionRads));
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return SwerveConversions
                .wheelRotationsPerSecToMetersPerSec(Units.radiansToRotations(inputs.driveVelocityRadsPerSec));
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle().minus(Rotation2d.fromRadians(inputs.chassisAngularOffset)));
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle().minus(Rotation2d.fromRadians(inputs.chassisAngularOffset)));
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drivePositionRads;
    }
}