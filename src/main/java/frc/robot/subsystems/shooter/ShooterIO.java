package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

    @AutoLog
    public static class ShooterIOInputs {
        public double motorAppliedVoltage = 0;
        public double motorCurrent = 0;
        public double motorVelocityRadiansPerSecond = 0;

        public double servoAngle;
    }

    public abstract void updateInputs(ShooterIOInputs inputs);

    public abstract void setVoltage(double volts);

    public abstract void setVelocityRadPerSec(double velocity);

    public default void setServoAngleRad(double radians) {}
}