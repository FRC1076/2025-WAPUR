package frc.robot.subsystems.yoinker;

import frc.robot.Constants.YoinkerConstants;
import frc.robot.subsystems.yoinker.YoinkerIO;
import lib.control.DynamicArmFeedforward;
import lib.control.DynamicProfiledPIDController;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
public class YoinkerSubsystem extends SubsystemBase
{
    private final YoinkerIO io;
    private final ProfiledPIDController m_profiledPIDController;
    private final DynamicArmFeedforward m_feedforwardController;
    private final YoinkerIOInputsAutoLogged inputs = new YoinkerIOInputsAutoLogged();
    private final SysIdRoutine sysid;
    

    public YoinkerSubsystem(YoinkerIO io, DoubleSupplier periodSupplier) 
    {
        this.io = io;

        var controlConstants = io.getControlConstants();

        m_profiledPIDController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(),
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new DynamicArmFeedforward(
            controlConstants.kS(),
            controlConstants.kG(),
            controlConstants.kV(),
            controlConstants.kA()
        );

        sysid = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, Volts.of(1), null,
                (state) -> Logger.recordOutput("Yoinker/SysIDState", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );

    }
    
    public void setVoltage(double volts) 
    {
        
        if (this.getAngleRadians() > YoinkerConstants.kMaxYoinkerAngleRadians && volts > 0) 
        {
            volts = 0;
        } else if (this.getAngleRadians() < YoinkerConstants.kMinYoinkerAngleRadians && volts < 0) 
        {
            volts = 0;
        }

        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    public void setAngle(Rotation2d position) 
    {
        io.setVoltage(
            m_profiledPIDController.calculate(inputs.angleRadians, MathUtil.clamp(position.getRadians(), YoinkerConstants.kMinYoinkerAngleRadians, YoinkerConstants.kMaxYoinkerAngleRadians))
            + m_feedforwardController.calculate(inputs.angleRadians, m_profiledPIDController.getSetpoint().velocity)
        );
    }

    public double getAngleRadians() 
    {
        return inputs.angleRadians;
    }

    public Rotation2d getAngle() 
    {
        return Rotation2d.fromRadians(inputs.angleRadians);
    }

    public void stop() 
    {
        setVoltage(0);
    }

    public void setKg(double kg) 
    {
        m_feedforwardController.setKg(kg);
    }

    public Command applyAngle(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getAngleRadians(),inputs.velocityRadiansPerSecond);},
            () -> setAngle(angle), 
            (interrupted) -> {},
            () -> Math.abs(angle.minus(getAngle()).getRadians()) < YoinkerConstants.yoinkerAngleToleranceRadians,
            this
        );
    }
    public boolean withinTolerance(double tolerance){
        return Math.abs(m_profiledPIDController.getGoal().position - getAngleRadians()) < tolerance;
    }

    
    public Command applyAnglePersistent(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getAngleRadians(),inputs.velocityRadiansPerSecond);
                    m_profiledPIDController.setGoal(angle.getRadians());},
            () -> setAngle(angle), 
            (interrupted) -> {},
            () -> false,
            this
        );
    }

    public Command holdAngle(Rotation2d angle)
    {
        return run(() -> setAngle(angle));
    }

    public Command applyManualControl(DoubleSupplier controlSupplier) 
    {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * YoinkerConstants.maxOperatorControlVolts));
    }

    @Override
    public void periodic() 
    {
        io.updateInputs(inputs);
        Logger.recordOutput("Yoinker/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Yoinker/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Yoinker", inputs);
    }

    @Override
    public void simulationPeriodic() 
    {
        io.simulationPeriodic();
    }

    public Command yoinkerSysIdQuasistatic(SysIdRoutine.Direction direction) 
    {
        return sysid.quasistatic(direction);
    }

    public Command yoinkerSysIdDynamic(SysIdRoutine.Direction direction) 
    {
        return sysid.dynamic(direction);
    }
}
