package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class ShooterSubsystem extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private final SysIdRoutine sysid;

    public ShooterSubsystem(ShooterIO io) {
        this.io = io;

        sysid = new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null,
                (state) -> Logger.recordOutput("Shooter/SysIDState", state.toString())
            ),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)),
                null,
                this
            )
        );
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
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

    public Command shooterSysIdQuasistatic(Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command shooterSysIdDynamic(Direction direction) {
        return sysid.dynamic(direction);
    }
}