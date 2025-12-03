package frc.robot.subsystems.grabber;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.GrabberConstants;

import org.littletonrobotics.junction.Logger;

public class GrabberSubsystem extends SubsystemBase {
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

    private final LinearFilter currentFilter;

    public GrabberSubsystem(GrabberIO io) {
        this.io = io;
        currentFilter = LinearFilter.movingAverage(GrabberConstants.kCurrentFilterTaps);
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
        inputs.filteredCurrent = currentFilter.calculate(inputs.leadMotorCurrentAmps);
        Logger.processInputs("Grabber", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command applyVoltage(double volts) {
        return Commands.runOnce(() -> setVoltage(volts), this);
    }

    /** Returns whether or not the current is at or above the desired amperage for a specified number of seconds
     * 
     * @param currentAmps The specified current
     * @param debounceSeconds How long it has to be at the current
     */
    public Trigger aboveCurrentDebounced(double currentAmps, double debounceSeconds) {
        return new Trigger(() -> inputs.filteredCurrent >= currentAmps).debounce(debounceSeconds);
    }
    
    /** Returns whether or not the current is at or below the desired amperage for a specified number of seconds
     * 
     * @param currentAmps The specified current
     * @param debounceSeconds How long it has to be at the current
     */
    public Trigger belowCurrentDebounced(double currentAmps, double debounceSeconds) {
        return new Trigger(() -> inputs.filteredCurrent <= currentAmps).debounce(debounceSeconds);
    }

    public double getAppliedCurrent() {
        return inputs.leadMotorCurrentAmps;
    }
}