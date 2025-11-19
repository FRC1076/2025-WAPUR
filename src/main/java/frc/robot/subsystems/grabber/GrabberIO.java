package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double appliedVolts = 0;
        public double leadMotorCurrentAmps = 0;

        public double followMotorCurrentAmps = 0;

        public double motorPositionRadians = 0;
        public double velocityRadPerSec = 0;
    }

    public abstract void updateInputs(GrabberIOInputs inputs);

    public abstract void setVoltage(double volts);

    public default double getOutputCurrent() {
        return -1;
    }

    public default void simulationPeriodic() {}
}