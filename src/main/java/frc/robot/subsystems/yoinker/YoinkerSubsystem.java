package frc.robot.subsystems.yoinker;

import frc.robot.Constants.YoinkerConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class YoinkerSubsystem extends SubsystemBase {
    private final YoinkerIO io;
    private final ProfiledPIDController m_profiledPIDController;
    private final ArmFeedforward m_feedforwardController;
    private final YoinkerIOInputsAutoLogged inputs = new YoinkerIOInputsAutoLogged();
    private final SysIdRoutine sysid;
    private boolean PIDEnabled = false;

    public YoinkerSubsystem(YoinkerIO io) {
        this.io = io;

        var controlConstants = io.getControlConstants();

        m_profiledPIDController = new ProfiledPIDController(
            controlConstants.kP(),
            controlConstants.kI(),
            controlConstants.kD(),
            controlConstants.kProfileConstraints()
        );

        m_feedforwardController = new ArmFeedforward(
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
    
    public void setVoltage(double volts) {
        
        if (this.getAngleRadians() > YoinkerConstants.kMaxYoinkerAngleRadians && volts > 0) {
            volts = 0;
        } else if (this.getAngleRadians() < YoinkerConstants.kMinYoinkerAngleRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    public double getAngleRadians() {
        return inputs.angleRadians;
    }

    public void stop() {
        setVoltage(0);
    }

    public boolean withinTolerance(double tolerance) {
        return Math.abs(m_profiledPIDController.getGoal().position - getAngleRadians()) < tolerance;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (PIDEnabled) {
            setVoltage(m_profiledPIDController.calculate(inputs.angleRadians));
        }
        
        Logger.recordOutput("Yoinker/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Yoinker/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Yoinker", inputs);

    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setPIDEnabled(boolean enabled) {
        this.PIDEnabled = enabled;
    }

    public void setPIDTargetRadians(double targetRadians) {
        m_profiledPIDController.setGoal(targetRadians);
    }

    public Command runPID(double targetRadians) {
        return Commands.sequence(
            Commands.runOnce(() -> setPIDTargetRadians(targetRadians)),
            Commands.runOnce(() -> setPIDEnabled(true))
        );
    }

    public Command disablePID() {
        return Commands.runOnce(() -> setPIDEnabled(false));
    }

    public Command yoinkerSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command yoinkerSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }
}
