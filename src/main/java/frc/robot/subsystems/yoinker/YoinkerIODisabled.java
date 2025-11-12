package frc.robot.subsystems.yoinker;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class YoinkerIODisabled implements YoinkerIO {
    private final YoinkerControlConstants disabledConstants = new YoinkerControlConstants(
        0.0, 0.0, 0.0, new Constraints(0, 0), 
        0.0, 0.0, 0.0, 0.0);
    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public YoinkerControlConstants getControlConstants() {
        return disabledConstants;
    }

    @Override
    public void updateInputs(YoinkerIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
    }
}
