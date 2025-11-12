package frc.robot.subsystems.wrist;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class WristIODisabled implements WristIO {
    private final WristControlConstants disabledConstants = new WristControlConstants(
        0.0, 0.0, 0.0, new Constraints(0, 0), 
        0.0, 0.0, 0.0, 0.0);
    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public WristControlConstants getControlConstants() {
        return disabledConstants;
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
    }
}
