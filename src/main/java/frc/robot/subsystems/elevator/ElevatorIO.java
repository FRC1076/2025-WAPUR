package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public interface ElevatorIO {
    public static record ElevatorControlConstants (
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
    public static class ElevatorIOInputs {

        public double appliedVolts = 0;
        public double appliedOutput = 0;
        public double leadCurrentAmps = 0;
        public double followCurrentAmps = 0;

        public double elevatorHeightMeters = 0;
        public double velocityMetersPerSecond = 0;
    }

    
    public abstract ElevatorControlConstants getControlConstants();
    public abstract void updateInputs(ElevatorIOInputs inputs);
    public abstract void setVoltage(double volts);
    public abstract void resetPosition(double positionMeters);
    public default void simulationPeriodic() {}

}