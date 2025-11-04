package frc.robot.subsystems.yoinker;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

import org.littletonrobotics.junction.AutoLog;

public interface YoinkerIO 
{
    public static record YoinkerControlConstants (
        Double kP,
        Double kI,
        Double kD,
        Constraints kProfileConstraints,
        Double kS,
        Double kG,
        Double kV,
        Double kA
    ) {}
    
    @AutoLog
    public static class YoinkerIOInputs 
    {
        public double appliedVolts = 0;
        public double leadCurrentAmps = 0;
        public double angleRadians = 0;
        public double velocityRadiansPerSecond = 0;
    }

    public abstract YoinkerControlConstants getControlConstants();

    public abstract void updateInputs(YoinkerIOInputs inputs);

    public abstract void setVoltage(double volts);

    public default void simulationPeriodic() {}
}
