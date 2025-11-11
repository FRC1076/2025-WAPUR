package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.SuperstructureConstants.BallStates;
import frc.robot.Constants.SuperstructureConstants.CrateStates;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.yoinker.YoinkerSubsystem;

public class Superstructure {
    
    public class MutableSuperState {
        BallStates ballState = BallStates.HOME;
        CrateStates crateState = CrateStates.HOME;

        public BallStates getBallState() {
            return this.ballState;
        }

        public void setBallState(BallStates ballState) {
            this.ballState = ballState;
        }

        public CrateStates getCrateState() {
            return this.crateState;
        }

        public void setCrateState(CrateStates crateState) {
            this.crateState = crateState;
        }

    }

    private ElevatorSubsystem m_elevator;
    private GrabberSubsystem m_grabber;
    private ShooterSubsystem m_shooter;
    private WristSubsystem m_wrist;
    private IntakeSubsystem m_intake;
    private YoinkerSubsystem m_yoinker;

    public Superstructure(
        ElevatorSubsystem elevatorSubsystem,
        GrabberSubsystem grabberSubsystem,
        ShooterSubsystem shooterSubsystem,
        WristSubsystem wristSubsystem,
        IntakeSubsystem intakeSubsystem,
        YoinkerSubsystem yoinkerSubsystem
        ) {
            m_elevator = elevatorSubsystem;
            m_grabber = grabberSubsystem;
            m_shooter = shooterSubsystem;
            m_wrist = wristSubsystem;
            m_intake = intakeSubsystem;
            m_yoinker = yoinkerSubsystem;
    }

    public Command setCrateStateAllParallel (CrateStates crateState) {
        return Commands.parallel(
            m_elevator.startPID(crateState.elevatorHeight),
            m_grabber.applyVoltage(crateState.grabberVoltage)
        );
    }

    public Command setCrateStateElevatorFirst (CrateStates crateState) {
        return Commands.sequence(
            m_elevator.startPID(crateState.elevatorHeight),
            m_grabber.applyVoltage(crateState.grabberVoltage)
        );
    }

    public Command setBallStateAllParallel (BallStates ballState) {
        return Commands.parallel(
            m_wrist.startPID(ballState.wristAngleRadians),
            m_shooter.startPID(ballState.shooterRadPerSec),
            m_intake.applyVoltage(ballState.intakeSpeed)
        );
    }
}


