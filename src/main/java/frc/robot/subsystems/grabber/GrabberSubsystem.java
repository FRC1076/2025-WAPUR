package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class GrabberSubsystem extends SubsystemBase {
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        this.io.setVoltage(volts);
    }

    public void stop() {
        setVoltage(0);
    }

    public double getPosition() {
        return inputs.motorPositionRadians;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }

    public Command applyVoltage(double volts) {
        return runOnce(() -> setVoltage(volts));
    }

    public double getAppliedCurrent() {
        return inputs.leftMotorCurrent;
    }
}