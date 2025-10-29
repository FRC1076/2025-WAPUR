package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;
import lib.control.DynamicElevatorFeedforward;
import lib.control.DynamicProfiledPIDController;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.filter.Debouncer;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase 
{

    public static final double homingVolts = -0.1;
    public static final double homingDebounceTime = 0.25;
    public static final double homingVelocityThreshold = 0.1;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final ProfiledPIDController m_profiledPIDController;
    private final DynamicElevatorFeedforward m_feedforwardController;
    
    private boolean homed = false;
    private Debouncer homingDebouncer;

    private final SysIdRoutine m_elevatorSysIdRoutine;

    public ElevatorSubsystem(ElevatorIO io, DoubleSupplier periodSupplier)
    {
        this.io = io;
        var controlConstants = io.getControlConstants();

        m_profiledPIDController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(),
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new DynamicElevatorFeedforward(
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
    public void periodic()
    {
        io.updateInputs(inputs);
        Logger.recordOutput("Elevator/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Elevator/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void simulationPeriodic()
    {
        io.simulationPeriodic();
    }
    
    public void setPosition(double positionMeters) 
    {
        io.setVoltage (
            m_profiledPIDController.calculate(getPositionMeters(), MathUtil.clamp(
                positionMeters, 
                ElevatorConstants.kMinElevatorHeightMeters, 
                ElevatorConstants.kMaxElevatorHeightMeters))
            + m_feedforwardController.calculate(m_profiledPIDController.getSetpoint().velocity)
        );
    }

    public void setVoltage(double volts) 
    {
        
        if (this.getPositionMeters() > ElevatorConstants.kMaxElevatorHeightMeters && volts > 0) {
            volts = 0;
        } else if (this.getPositionMeters() < ElevatorConstants.kMinElevatorHeightMeters && volts < 0) {
            volts = 0;
        }
    
        io.setVoltage(volts + m_feedforwardController.getKg());
    }

    public void setVoltageUnrestricted(double volts) 
    {
        io.setVoltage(volts + m_feedforwardController.getKg());
    }

    public void setKg(double kg) 
    {
        m_feedforwardController.setKg(kg);
    }

    public double getPositionMeters()
    {
        return inputs.elevatorHeightMeters;
    }

    public boolean isZeroed() 
    {
        return homed;
    }

    public boolean withinTolerance(double tolerance)
    {
        return Math.abs(m_profiledPIDController.getGoal().position - getPositionMeters()) < tolerance;
    }

    public Command applyPosition(double positionMeters) 
    {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getPositionMeters(), inputs.velocityMetersPerSecond);},
            () -> setPosition(positionMeters),
            (interrupted) -> {},
            () -> Math.abs(positionMeters - getPositionMeters()) < ElevatorConstants.elevatorPositionToleranceMeters,
            this
        );
    }

    public Command holdPosition(double positionMeters) 
    {
        return run(() -> setPosition(positionMeters));
    }

    public Command applyPositionPersistent(double positionMeters)
    {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getPositionMeters(), inputs.velocityMetersPerSecond);
                    m_profiledPIDController.setGoal(positionMeters);},
            () -> setPosition(positionMeters),
            (interrupted) -> {},
            () -> false,
            this
        );
    }
    
    public Command applyManualControl(DoubleSupplier controlSupplier, BooleanSupplier higherMaxSpeedSupplier) 
    {
        return run(higherMaxSpeedSupplier.getAsBoolean()
            ? () -> setVoltageUnrestricted(controlSupplier.getAsDouble() * ElevatorConstants.fasterMaxOperatorControlVolts)
            : () -> setVoltageUnrestricted(controlSupplier.getAsDouble() * ElevatorConstants.defaultMaxOperatorControlVolts)
        );
    }

    public Command zeroEncoderJoystickControl(DoubleSupplier controlSupplier) 
    {
        return run(() -> io.setVoltage(controlSupplier.getAsDouble() * ElevatorConstants.defaultMaxOperatorControlVolts))
            .finallyDo(() -> io.resetPosition(0));
    }

    public Command elevatorSysIdQuasistatic(SysIdRoutine.Direction direction) 
    {
        return m_elevatorSysIdRoutine.quasistatic(direction);
    }

    public Command elevatorSysIdDynamic(SysIdRoutine.Direction direction) 
    {
        return m_elevatorSysIdRoutine.dynamic(direction);
    }

    public Command autoHome() 
    {
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