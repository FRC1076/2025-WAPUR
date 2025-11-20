package frc.robot.subsystems.shooter;

public class ShooterIODisabled implements ShooterIO {

    private double voltageTarget = 0;

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;        
    }

    

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = voltageTarget;
    }
}
