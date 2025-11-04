package frc.robot.subsystems.wrist;

import frc.robot.Constants.WristConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class WristSubsystem extends SubsystemBase {
    private final WristIO io;
    private final ProfiledPIDController m_profiledPIDController;
    private final ArmFeedforward m_feedforwardController;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
    private final SysIdRoutine sysid;
    

    public WristSubsystem(WristIO io, DoubleSupplier periodSupplier) {
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

    public void setAngle(Rotation2d position) {
        io.setVoltage(
            m_profiledPIDController.calculate(inputs.angleRadians, MathUtil.clamp(position.getRadians(), WristConstants.kMinWristAngleRadians, WristConstants.kMaxWristAngleRadians))
            + m_feedforwardController.calculate(inputs.angleRadians, m_profiledPIDController.getSetpoint().velocity)
        );
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

    public Command applyAngle(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {m_profiledPIDController.reset(getAngleRadians(),inputs.velocityRadiansPerSecond);},
            () -> setAngle(angle), 
            (interrupted) -> {},
            () -> Math.abs(angle.minus(getAngle()).getRadians()) < WristConstants.wristAngleToleranceRadians,
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

    public Command holdAngle(Rotation2d angle) {
        return run(() -> setAngle(angle));
    }

    public Command applyManualControl(DoubleSupplier controlSupplier) {
        return run(() -> setVoltage(controlSupplier.getAsDouble() * WristConstants.maxOperatorControlVolts));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Wrist/Setpoint", m_profiledPIDController.getSetpoint().position);
        Logger.recordOutput("Wrist/VelocitySetpoint", m_profiledPIDController.getSetpoint().velocity);
        Logger.processInputs("Wrist", inputs);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }

    public Command wristSysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command wristSysIdDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }
}