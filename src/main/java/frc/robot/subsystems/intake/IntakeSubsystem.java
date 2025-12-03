package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public IntakeSubsystem(IntakeIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        this.io.setVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command applyVoltage(double volts) {
        return Commands.runOnce(() -> setVoltage(volts), this);
    }
}
