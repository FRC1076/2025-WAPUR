package frc.robot.subsystems;

import java.util.HashMap;

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

    private MutableSuperState m_superState;

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
            m_superState = new MutableSuperState();
    }

    public Command setCrateStateAllParallel (CrateStates crateState) {
        m_superState.setCrateState(crateState);
        return Commands.parallel(
            m_elevator.startPID(crateState.elevatorHeight),
            m_grabber.applyVoltage(crateState.grabberVoltage)
        );
    }

    public Command setCrateStateNoElevator (CrateStates crateState) {
        m_superState.setCrateState(crateState);
        return m_grabber.applyVoltage(crateState.grabberVoltage);
    }

    public Command setCrateStateElevatorFirst (CrateStates crateState) {
        m_superState.setCrateState(crateState);
        return Commands.sequence(
            m_elevator.startPID(crateState.elevatorHeight),
            m_grabber.applyVoltage(crateState.grabberVoltage)
        );
    }

    public Command setBallStateAllParallel (BallStates ballState) {
        m_superState.setBallState(ballState);
        return Commands.parallel(
            m_wrist.startPID(ballState.wristAngleRadians),
            m_shooter.startPID(ballState.shooterRadPerSec),
            m_intake.applyVoltage(ballState.intakeSpeed)
        );
    }

    public Command setBallStateWristFirst (BallStates ballState) {
        m_superState.setBallState(ballState);
        return Commands.sequence(
            m_wrist.startPID(ballState.wristAngleRadians),
            Commands.parallel(
                m_shooter.startPID(ballState.shooterRadPerSec),
                m_intake.applyVoltage(ballState.intakeSpeed)
            )
        );
    }

    public class SuperstructureCommandFactory {
        private final HashMap <CrateStates, CrateStates> preToScoringStates = new HashMap<CrateStates, CrateStates>();

        public SuperstructureCommandFactory() {
            preToScoringStates.put(CrateStates.PRE_L1, CrateStates.SHOOT_L1);
            preToScoringStates.put(CrateStates.PRE_L2, CrateStates.SHOOT_L2);
            preToScoringStates.put(CrateStates.PRE_L3, CrateStates.SHOOT_L3);
            preToScoringStates.put(CrateStates.PRE_L4, CrateStates.SHOOT_L4);
        }

        public Command homeCrates() {
            return setCrateStateAllParallel(CrateStates.HOME);
        }

        public Command intakeCrate() {
            return setCrateStateElevatorFirst(CrateStates.INTAKE_CRATES);
        }

        public Command preL1(){
            return setCrateStateAllParallel(CrateStates.PRE_L1);
        }

        public Command preL2(){
            return setCrateStateAllParallel(CrateStates.PRE_L2);
        }

        public Command preL3(){
            return setCrateStateAllParallel(CrateStates.PRE_L3);
        }

        public Command preL4(){
            return setCrateStateAllParallel(CrateStates.PRE_L4);
        }

        public Command shootCrate(){
            return setCrateStateNoElevator(preToScoringStates.getOrDefault(m_superState.getCrateState(),CrateStates.SHOOT_L1));
        }

        public Command homeBalls(){
            return setBallStateAllParallel(BallStates.HOME);
        }

        public Command intakeDownBalls(){
            return setBallStateAllParallel(BallStates.INTAKE_DOWN);
        }

        public Command intakeBalls(){
            return setBallStateWristFirst(BallStates.INTAKING);
        }

        public Command shootBalls(){
            return setBallStateAllParallel(BallStates.SHOOT);
        }

        public Command shootBallsWristUp(){
            return setBallStateWristFirst(BallStates.SHOOT_WRIST_UP);
        }
    }
}


