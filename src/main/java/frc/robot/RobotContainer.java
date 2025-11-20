// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.GrabberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.DriveConstants.ModuleConstants.ModuleConfig;
import frc.robot.Constants.OIConstants.OperatorControllerStates;
import frc.robot.Constants.SystemConstants.RobotMode;
import frc.robot.commands.drive.TeleopDriveCommandV2;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SuperstructureVisualizer;
import frc.robot.subsystems.Superstructure.SuperstructureCommandFactory;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.GyroIODisabled;
import frc.robot.subsystems.drive.GyroIOPigeon;
import frc.robot.subsystems.drive.ModuleIODisabled;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.ElevatorIODisabled;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberIODisabled;
import frc.robot.subsystems.grabber.GrabberIOHardware;
import frc.robot.subsystems.grabber.GrabberIOSim;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.intake.IntakeIODisabled;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIODisabled;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristIODisabled;
import frc.robot.subsystems.wrist.WristIOHardware;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.yoinker.YoinkerIODisabled;
import frc.robot.subsystems.yoinker.YoinkerIOHardware;
import frc.robot.subsystems.yoinker.YoinkerIOSim;
import frc.robot.subsystems.yoinker.YoinkerSubsystem;
import lib.hardware.hid.SamuraiXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_drive;
    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IntakeSubsystem m_intake;
    private final ShooterSubsystem m_shooter;
    private final WristSubsystem m_wrist;
    private final YoinkerSubsystem m_yoinker;

    private final Superstructure m_superstructure;
    private final SuperstructureVisualizer m_superVis;

    // Drive command
    private final TeleopDriveCommandV2 driveCommand;

    // Controllers
    private final SamuraiXboxController m_driverController =
        new SamuraiXboxController(OIConstants.kDriverControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);
    private final SamuraiXboxController m_operatorController = 
        new SamuraiXboxController(OIConstants.kOperatorControllerPort)
            .withDeadband(OIConstants.kControllerDeadband)
            .withTriggerThreshold(OIConstants.kControllerTriggerThreshold);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        if (SystemConstants.currentMode == RobotMode.REAL) {
            m_drive = new DriveSubsystem(
                new GyroIOPigeon(), 
                new ModuleIOHardware(ModuleConfig.FrontLeft),
                new ModuleIOHardware(ModuleConfig.FrontRight), 
                new ModuleIOHardware(ModuleConfig.RearLeft),
                new ModuleIOHardware(ModuleConfig.RearRight)
            );
            m_elevator = new ElevatorSubsystem(new ElevatorIOHardware());
            m_grabber = new GrabberSubsystem(new GrabberIOHardware());
            m_intake = new IntakeSubsystem(new IntakeIOHardware());
            m_shooter = new ShooterSubsystem(new ShooterIOHardware());
            m_wrist = new WristSubsystem(new WristIOHardware());
            m_yoinker = new YoinkerSubsystem(new YoinkerIOHardware());
        } else if (SystemConstants.currentMode == RobotMode.REAL_NO_MECHANISMS) {
            m_drive = new DriveSubsystem(
                new GyroIOPigeon(), 
                new ModuleIOHardware(ModuleConfig.FrontLeft),
                new ModuleIOHardware(ModuleConfig.FrontRight), 
                new ModuleIOHardware(ModuleConfig.RearLeft),
                new ModuleIOHardware(ModuleConfig.RearRight)
            );
            m_elevator = new ElevatorSubsystem(new ElevatorIODisabled());
            m_grabber = new GrabberSubsystem(new GrabberIODisabled());
            m_intake = new IntakeSubsystem(new IntakeIODisabled());
            m_shooter = new ShooterSubsystem(new ShooterIODisabled());
            m_wrist = new WristSubsystem(new WristIODisabled());
            m_yoinker = new YoinkerSubsystem(new YoinkerIODisabled());
        } else if (SystemConstants.currentMode == RobotMode.REAL_NO_DRIVETRAIN) {
            m_drive = new DriveSubsystem(
                new GyroIODisabled(), 
                new ModuleIODisabled(),
                new ModuleIODisabled(), 
                new ModuleIODisabled(),
                new ModuleIODisabled()
            );
            m_elevator = new ElevatorSubsystem(new ElevatorIOHardware());
            m_grabber = new GrabberSubsystem(new GrabberIOHardware());
            m_intake = new IntakeSubsystem(new IntakeIOHardware());
            m_shooter = new ShooterSubsystem(new ShooterIOHardware());
            m_wrist = new WristSubsystem(new WristIOHardware());
            m_yoinker = new YoinkerSubsystem(new YoinkerIOHardware());
        } else if (SystemConstants.currentMode == RobotMode.SIM) {
            m_drive = new DriveSubsystem(
                new GyroIODisabled(), 
                new ModuleIOSim(),
                new ModuleIOSim(), 
                new ModuleIOSim(),
                new ModuleIOSim()
            );
            m_elevator = new ElevatorSubsystem(new ElevatorIOSim());
            m_grabber = new GrabberSubsystem(new GrabberIOSim());
            m_intake = new IntakeSubsystem(new IntakeIOSim());
            m_shooter = new ShooterSubsystem(new ShooterIOSim());
            m_wrist = new WristSubsystem(new WristIOSim());
            m_yoinker = new YoinkerSubsystem(new YoinkerIOSim());
        } else {
            // All disabled
            m_drive = new DriveSubsystem(
                new GyroIODisabled(), 
                new ModuleIODisabled(),
                new ModuleIODisabled(), 
                new ModuleIODisabled(),
                new ModuleIODisabled()
            );
            m_elevator = new ElevatorSubsystem(new ElevatorIODisabled());
            m_grabber = new GrabberSubsystem(new GrabberIODisabled());
            m_intake = new IntakeSubsystem(new IntakeIODisabled());
            m_shooter = new ShooterSubsystem(new ShooterIODisabled());
            m_wrist = new WristSubsystem(new WristIODisabled());
            m_yoinker = new YoinkerSubsystem(new YoinkerIODisabled());
        }

        m_superstructure = new Superstructure(
            m_elevator,
            m_grabber,
            m_shooter,
            m_wrist,
            m_intake,
            m_yoinker
        );
         
        m_superVis = new SuperstructureVisualizer(m_superstructure);

        driveCommand = m_drive.CommandBuilder.driveTeleop(
            () -> -m_driverController.getLeftY(), 
            () -> -m_driverController.getLeftX(), 
            () -> -m_driverController.getRightX(), 
            1, 
            1, 
            DriveConstants.useSpeedScaling
        );

        m_drive.setDefaultCommand(driveCommand);

        // Configure controller bindings
        configureDriverBindings();

        configureOperatorBindings();
    }

    /** Maps triggers on driver controller to commands */
    private void configureDriverBindings() {
        final SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandFactory();

        // Apply single clutch
        m_driverController.leftBumper()
            .whileTrue(driveCommand.applySingleClutch());

        // Apply double clutch
        m_driverController.rightBumper()
            .whileTrue(driveCommand.applyDoubleClutch());

        // Re-zero gyro
        m_driverController.start()
            .onTrue(Commands.runOnce(() -> m_drive.rezeroGyro()));

        m_driverController.leftTrigger()
            .onTrue(superstructureCommands.intakeCrate());
        
        m_driverController.x()
            .onTrue(superstructureCommands.preL1());
        
        m_driverController.a()
            .onTrue(superstructureCommands.preL2());

        m_driverController.b()
            .onTrue(superstructureCommands.preL3());

        m_driverController.y()
            .onTrue(superstructureCommands.preL4());

        m_driverController.rightTrigger()
            .onTrue(superstructureCommands.shootCrate());

        m_driverController.leftBumper()
            .onTrue(superstructureCommands.intakeBalls());

        m_driverController.rightBumper()
            .onTrue(superstructureCommands.shootBalls());
    
        
    }

    /** Maps triggers on the operator controller to commands */
    private void configureOperatorBindings() {
        SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandFactory();

        m_operatorController.leftActive()
            .whileTrue(
                m_elevator.applyVoltageUnrestricted(
                    m_operatorController.getLeftY()
                     * ElevatorConstants.defaultMaxOperatorControlVolts)
            )
            .onFalse(m_elevator.applyVoltageUnrestricted(0));

        m_operatorController.rightActive()
            .whileTrue( 
                m_wrist.applyVoltageUnrestricted(
                    m_operatorController.getRightY() 
                        * WristConstants.maxOperatorControlVolts)
            )
            .onFalse(m_elevator.applyVoltageUnrestricted(0));

        m_operatorController.povUp()
            .whileTrue(m_grabber.applyVoltage(GrabberConstants.kOperatorControlVolts))
            .onFalse(m_grabber.applyVoltage(0));

        m_operatorController.povDown()
            .whileTrue(m_grabber.applyVoltage(-GrabberConstants.kOperatorControlVolts))
            .onFalse(m_grabber.applyVoltage(0));

        m_operatorController.povRight()
            .whileTrue(superstructureCommands.manualBallsForward());

        m_operatorController.povLeft()
            .whileTrue(superstructureCommands.manualBallsBackward());

        if (OIConstants.kOperatorControllerState == OperatorControllerStates.OPERATOR) {
            // Operate I guess
        } else if (OIConstants.kOperatorControllerState == OperatorControllerStates.DRIVETRAIN_SYSID_TRANS) {
            m_operatorController.a()
                .and(m_operatorController.x())
                .whileTrue(m_drive.CommandBuilder.sysIdQuasistaticTranslation(Direction.kForward));

            m_operatorController.a()
                .and(m_operatorController.y())
                .whileTrue(m_drive.CommandBuilder.sysIdQuasistaticTranslation(Direction.kReverse));

            m_operatorController.b()
                .and(m_operatorController.x())
                .whileTrue(m_drive.CommandBuilder.sysIdDyanmicTranslation(Direction.kForward));

            m_operatorController.b()
                .and(m_operatorController.y())
                .whileTrue(m_drive.CommandBuilder.sysIdDyanmicTranslation(Direction.kReverse));
        } else if (OIConstants.kOperatorControllerState == OperatorControllerStates.DRIVETRAIN_SYSID_SPIN) {
            m_operatorController.a()
                .and(m_operatorController.x())
                .whileTrue(m_drive.CommandBuilder.sysIdQuasistaticSpin(Direction.kForward));

            m_operatorController.a()
                .and(m_operatorController.y())
                .whileTrue(m_drive.CommandBuilder.sysIdQuasistaticSpin(Direction.kReverse));

            m_operatorController.b()
                .and(m_operatorController.x())
                .whileTrue(m_drive.CommandBuilder.sysIdDyanmicSpin(Direction.kForward));

            m_operatorController.b()
                .and(m_operatorController.y())
                .whileTrue(m_drive.CommandBuilder.sysIdDyanmicSpin(Direction.kReverse));
        }
    }

    public void configureNamedCommands() {
        SuperstructureCommandFactory superstructureCommands = m_superstructure.getCommandFactory();
        NamedCommands.registerCommand("Shoot Balls", superstructureCommands.shootBallsWristUp());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return AutoBuilder.buildAuto("Standard Auton Left");
    }

    /** Raise thread priority to reduce loop times */
    public static Command threadCommand() {
        return Commands.sequence(
                Commands.waitSeconds(20),
                Commands.runOnce(() -> Threads.setCurrentThreadPriority(true, 1)),
                Commands.print("Main Thread Priority raised to RT1 at " + Timer.getFPGATimestamp()))
            .ignoringDisable(true);
    }
}
