package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
        public double leftMotorAppliedVoltage = 0;
        public double leftMotorCurrent = 0;
        public double leftMotorRPM = 0;

        public double rightMotorAppliedVoltage = 0;
        public double rightMotorCurrent = 0;
        public double rightMotorRPM = 0;

        public double motorPositionRadians = 0;
    }

    public abstract void updateInputs(GrabberIOInputs inputs);

    public abstract void setVoltage(double volts);

    public default double getOutputCurrent() {
        return -1;
    }
}