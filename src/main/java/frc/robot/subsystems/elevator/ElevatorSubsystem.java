package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase  {

    public static final double homingVolts = -0.1;
    public static final double homingDebounceTime = 0.25;
    public static final double homingVelocityThreshold = 0.1;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController m_profiledPIDController;
    private final ElevatorFeedforward m_feedforwardController;
    
    private boolean homed = false;
    private Debouncer homingDebouncer;

    private final SysIdRoutine m_elevatorSysIdRoutine;

    private boolean PIDEnabled;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
        var controlConstants = io.getControlConstants();

        m_profiledPIDController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(),
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new ElevatorFeedforward(
            controlConstants.kS(),
            controlConstants.kG(),
            controlConstants.kV(),
            controlConstants.kA()
        );

        m_elevatorSysIdRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Volts.of(2.5),
                null,
                (state) -> Logger.recordOutput("Elevator/SysIDState", state.toString())
            ) , 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if(PIDEnabled) {
            setVoltage(m_profiledPIDController.calculate(inputs.elevatorHeightMeters));
        }
        Logger.recordOutput("Elevator/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Elevator/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
    

    public void setVoltage(double volts) {
        
        if (this.getPositionMeters() > ElevatorConstants.kMaxElevatorHeightMeters && volts > 0) {
            volts = 0;
        } else if (this.getPositionMeters() < ElevatorConstants.kMinElevatorHeightMeters && volts < 0) {
            volts = 0;
        }
    
        io.setVoltage(volts + m_feedforwardController.getKg());
    }

    public void setVoltageUnrestricted(double volts) {
        io.setVoltage(volts + m_feedforwardController.getKg());
    }

    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }

    public double getPositionMeters() {
        return inputs.elevatorHeightMeters;
    }

    public boolean isZeroed() {
        return homed;
    }

    public boolean withinTolerance(double tolerance) {
        return Math.abs(m_profiledPIDController.getGoal().position - getPositionMeters()) < tolerance;
    }

    public void setPIDEnabled(boolean enabled) {
        this.PIDEnabled = enabled;
    }

    public Command diablePID() {
        return Commands.runOnce(() -> setPIDEnabled(false));
    }

    public Command startPID(double targetMeters) {
        return Commands.sequence(
            Commands.runOnce(() -> m_profiledPIDController.setGoal(targetMeters)),
            Commands.runOnce(() -> setPIDEnabled(true))
        );
    }
    
    public Command applyManualControl(DoubleSupplier controlSupplier, BooleanSupplier higherMaxSpeedSupplier) {
        return run(higherMaxSpeedSupplier.getAsBoolean()
            ? () -> setVoltageUnrestricted(controlSupplier.getAsDouble() * ElevatorConstants.fasterMaxOperatorControlVolts)
            : () -> setVoltageUnrestricted(controlSupplier.getAsDouble() * ElevatorConstants.defaultMaxOperatorControlVolts)
        );
    }

    public Command zeroEncoderJoystickControl(DoubleSupplier controlSupplier) {
        return run(() -> io.setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.defaultMaxOperatorControlVolts))
            .finallyDo(() -> io.resetPosition(0));
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) {
        return m_elevatorSysIdRoutine.dynamic(direction);
    }

    public Command autoHome() {
        return startRun(
            () -> {
                homed = false;
                homingDebouncer = new Debouncer(homingDebounceTime);
                homingDebouncer.calculate(false);
                io.setVoltage(homingVolts);
            },
            () -> {
                homed = homingDebouncer.calculate(Math.abs(inputs.velocityMetersPerSecond) <= homingVelocityThreshold);
            }
        )
        .until(() -> homed)
        .andThen(() -> io.setVoltage(0))
        .finallyDo(() -> io.resetPosition(0));
    }
}