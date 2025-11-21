package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public Command applyVoltage(double volts) {
        return Commands.runOnce(() -> io.setVoltage(volts), this);
    }

    public Command applyVelocityRadPerSec(double velocity) {
        return Commands.runOnce(() -> io.setVelocityRadPerSec(velocity), this);
    }

    public Command applyServoAngle(double angleRadians) {
        return Commands.runOnce(() -> io.setServoAngleRad(angleRadians));
    }

    @Override
    public void periodic() {
        Logger.processInputs("Shooter", inputs);
    }
}