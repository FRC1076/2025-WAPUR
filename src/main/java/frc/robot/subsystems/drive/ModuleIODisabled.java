package frc.robot.subsystems.drive;

public class ModuleIODisabled implements ModuleIO {
    private double velocitySetpoint;
    private double turnSetpoint;

    @Override
    public void setDriveVelocity(double velocity, double FFVolts) {
        velocitySetpoint = velocity;
    }

    @Override
    public void setTurnPosition(double positionRadians, double FFVolts) {
        turnSetpoint = positionRadians;
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.drivePositionMeters = velocitySetpoint;
        inputs.turnPosition = turnSetpoint;
    }
}
