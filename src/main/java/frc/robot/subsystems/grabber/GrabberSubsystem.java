package frc.robot.subsystems.grabber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command applyVoltage(double volts) {
        return runOnce(() -> setVoltage(volts));
    }

    /** Returns whether or not the current is at or above the desired amperage for a specified number of seconds
     * 
     * @param currentAmps The specified current
     * @param debounceSeconds How long it has to be at the current
     */
    public Trigger aboveCurrentDebounced(double currentAmps, double debounceSeconds) {
        return new Trigger(() -> inputs.leadMotorCurrentAmps >= currentAmps).debounce(debounceSeconds);
    }
    
    /** Returns whether or not the current is at or below the desired amperage for a specified number of seconds
     * 
     * @param currentAmps The specified current
     * @param debounceSeconds How long it has to be at the current
     */
    public Trigger belowCurrentDebounced(double currentAmps, double debounceSeconds) {
        return new Trigger(() -> inputs.leadMotorCurrentAmps <= currentAmps).debounce(debounceSeconds);
    }

    public double getAppliedCurrent() {
        return inputs.leadMotorCurrentAmps;
    }
}