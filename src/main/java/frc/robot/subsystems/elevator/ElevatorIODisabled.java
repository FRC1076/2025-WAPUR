package frc.robot.subsystems.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ElevatorIODisabled implements ElevatorIO {
    private static final ElevatorControlConstants disabledConstants = new ElevatorControlConstants(
        0.0, 0.0, 0.0, new Constraints(0, 0),
        0.0, 0.0, 0.0, 0.0);

    private double voltageTarget = 0;

    @Override
    public ElevatorControlConstants getControlConstants() {
        return disabledConstants;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.appliedVolts = voltageTarget;
    }

    @Override
    public void setVoltage(double volts) {
        voltageTarget = volts;
    }

    @Override
    public void resetPosition(double positionMeters) {
        return;
    }
}
