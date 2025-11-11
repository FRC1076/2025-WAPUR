package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double appliedVoltage = 0;
        public double currentAmps = 0;
    }

    public abstract void setVoltage(double volts);

    public abstract void updateInputs(IntakeIOInputs inputs);
}
