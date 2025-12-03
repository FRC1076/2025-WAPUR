package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final ProfiledPIDController m_profiledPIDController;
    private final ArmFeedforward m_feedforwardController;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final SysIdRoutine sysid;
    private boolean PIDEnabled = false;
    

    public WristSubsystem(WristIO io) {
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
                (state) -> Logger.recordOutput("Wrist/SysIDState", state.toString())
            ), 
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );

    }
    
    public void setVoltage(double volts) {        
        if (this.getAngleRadians() > WristConstants.kMaxWristAngleRadians && volts > 0) {
            volts = 0;
        } else if (this.getAngleRadians() < WristConstants.kMinWristAngleRadians && volts < 0) {
            volts = 0;
        }

        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    public void setVoltageUnrestricted(double volts) {
        io.setVoltage(volts + m_feedforwardController.calculate(inputs.angleRadians, 0));
    }

    public double getAngleRadians() {
        return inputs.angleRadians;
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(inputs.angleRadians);
    }

    public void stop() {
        setVoltage(0);
    }

    public void setKg(double kg) {
        m_feedforwardController.setKg(kg);
    }
    
    public boolean withinTolerance(double tolerance){
        return Math.abs(m_profiledPIDController.getGoal().position - getAngleRadians()) < tolerance;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        if(PIDEnabled){
            setVoltage(m_profiledPIDController.calculate(inputs.angleRadians));
        }
        Logger.recordOutput("Wrist/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Wrist/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Wrist", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public void setPIDEnabled(boolean enabled) {
        this.PIDEnabled = enabled;
    }

    public Command startPID(double targetRadians) {
        return Commands.sequence(
            Commands.runOnce(() -> m_profiledPIDController.setGoal(targetRadians)),
            Commands.runOnce(() -> setPIDEnabled(true))
        ); 
    }

    public Command disablePID() {
        return Commands.runOnce(() -> setPIDEnabled(false));
    }

    public Command applyVoltageUnrestricted(double volts) {
        return Commands.sequence(
            disablePID(),
            Commands.runOnce(() -> setVoltageUnrestricted(volts), this)
        );
    } 

    public Command runVoltageUnrestricted(DoubleSupplier volts) {
        return Commands.sequence(
            disablePID(),
            Commands.run(() -> setVoltageUnrestricted(volts.getAsDouble()), this)
        );
    }

    public Command wristSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command wristSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }
}