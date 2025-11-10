package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;
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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
    }
    
}