package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    public static record ShooterControlConstants (
        Double kP,
        Double kI,
        Double kD,
        // No profile constraints for Shooter
        Double kS,
        Double kV,
        Double kA
    ) {}

    @AutoLog
    public static class ShooterIOInputs {
        public double motorAppliedVoltage = 0;
        public double motorCurrent = 0;
        public double motorVelocityRadiansPerSecond = 0;

        public double servoAngle;
    }

    public abstract ShooterControlConstants getShooterControlConstants();

    public abstract void updateInputs(ShooterIOInputs inputs);

    public abstract void setVoltage(double volts);

    public default void setServoAngleDeg(double degrees) {}
}