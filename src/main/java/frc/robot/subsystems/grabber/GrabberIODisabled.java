package frc.robot.subsystems.grabber;

public class GrabberIODisabled implements GrabberIO {
    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
    }
}
