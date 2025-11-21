package frc.robot.subsystems.shooter;

public class ShooterIODisabled implements ShooterIO {

    private double voltageTarget = 0;
    private double velocityTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;        
    }

    @Override
    public void setVelocityRadPerSec(double velocity) {
        if (velocity != 0) {
            voltageTarget = 12;
        } else {
            voltageTarget = 0;
        }
        velocityTarget = velocity;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = voltageTarget;
        inputs.motorVelocityRadiansPerSecond = velocityTarget;
    }
}
