package frc.robot.subsystems.shooter;

public class ShooterIODisabled implements ShooterIO {
    private final ShooterControlConstants disabledConstants = new ShooterControlConstants(
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0);
    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;        
    }

    @Override
    public ShooterControlConstants getShooterControlConstants() {
        return disabledConstants;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = voltageTarget;
    }
}
