package frc.robot.subsystems;

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

    ElevatorSubsystem m_elevator;
    GrabberSubsystem m_grabber;
    ShooterSubsystem m_shooter;
    WristSubsystem m_wrist;
    YoinkerSubsystem m_yoinker;

    public Superstructure(
        ElevatorSubsystem elevatorSubsystem,
        GrabberSubsystem grabberSubsystem,
        ShooterSubsystem shooterSubsystem,
        WristSubsystem wristSubsystem,
        YoinkerSubsystem yoinkerSubsystem
        ) {
            m_elevator = elevatorSubsystem;
            m_grabber = grabberSubsystem;
            m_shooter = shooterSubsystem;
            m_wrist = wristSubsystem;
            m_yoinker = yoinkerSubsystem;
    }

    public Command setCrateStateAllParallel (CrateState crateState) {
        return Commands.parallel(
            m_elevator.setPIDTarget(crateState.elevatorHeight),
            m_grabber.setVoltage(crateState.grabberVoltage)
        );
    }

    public Command setCrateStateElevatorFirst (CrateState crateState) {
        return Commands.sequence(
            m_elevator.setPIDTarget(crateState.elevatorHeight),
            m_grabber.setVoltage(crateState.grabberVoltage)
        );
    }

    public Command setBallStateAllParallel (BallState ballState) {
        return Commands.parallel(
            m_wrist.setPIDTarget(ballState.wristAngleRadians),
            m_shooter.setPIDTarget(ballState.shooterRadPerSec),
            m_intake.setPIDTarget(ballState.intakeSpeed)
        );
    }
}


