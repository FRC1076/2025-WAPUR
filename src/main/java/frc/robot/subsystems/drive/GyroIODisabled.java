package frc.robot.subsystems.drive;

public class GyroIODisabled implements GyroIO {
    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = false;
    }

    @Override
    public void reset() {
        return;
    }
}
