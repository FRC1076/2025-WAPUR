package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Voltage;

public interface ShooterIO 
{
    @AutoLog
    public static class ShooterIOInputs 
    {
        public double motorAppliedVoltage = 0;
        public double motorCurrent = 0;
        public double servoAppliedVoltage = 0;
        public double servoCurrent = 0;
        public double servoAngle;
    }

    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void runVolts(double volts) {}

    public default void runVolts(Measure<Voltage> voltage) 
    {
        runVolts(voltage.in(Volts));
    }

    public default void setServoAngleDeg(double degrees) {}
}