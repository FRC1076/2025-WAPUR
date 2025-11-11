package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final PIDController m_pidController;
    private final SimpleMotorFeedforward m_ffController;
    private boolean pidRunning;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
        
        var controlConstants = io.getShooterControlConstants();

        m_pidController = new PIDController(
            controlConstants.kP(), controlConstants.kI(), controlConstants.kD());
        
        m_ffController = new SimpleMotorFeedforward(
            controlConstants.kS(), controlConstants.kV(), controlConstants.kA());
    }

    public void runVolts(double volts) {
        this.io.runVolts(volts);
    }

    public void setServoAngleDeg(double degrees) {
        System.out.println(degrees);
        this.io.setServoAngleDeg(degrees);
    }

    public void stop() {
        runVolts(0);
    }

    public void setPidRunning(boolean enabled) {
        pidRunning = enabled;
    }

    public void setPidTarget(double targetRadPerSec) {
        m_pidController.setSetpoint(targetRadPerSec);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (pidRunning) {
            runVolts(m_pidController.calculate(inputs.motorVelocityRadiansPerSecond) + m_ffController.calculate(m_pidController.getSetpoint()));
        }

        Logger.processInputs("Shooter", inputs);
    }

    public Command startPID(double targetRadPerSec) {
        return Commands.sequence(
            Commands.runOnce(() -> setPidTarget(targetRadPerSec)),
            Commands.runOnce(() -> setPidRunning(true))
        );
    }

    public Command stopPID() {
        return Commands.runOnce(() -> setPidRunning(false));
    }
}