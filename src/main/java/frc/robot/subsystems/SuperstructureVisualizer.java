// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.wpilibj.util.Color8Bit;
import lib.extendedcommands.CommandUtils;

/*
 * This class is used to create mechanism visualizations and pass them into IOSim objects
 */
public class SuperstructureVisualizer {
    
    private final LoggedMechanism2d superstructureVis;

    private final LoggedMechanismRoot2d elevatorRoot;
    private final LoggedMechanismLigament2d elevator;
    private final LoggedMechanismLigament2d fixOnElevator;
    @SuppressWarnings("unused") // Needs to exist for visualization but isn't actually used anywhere
    private final LoggedMechanismLigament2d grabber;

    private final LoggedMechanismRoot2d wristRoot;
    private final LoggedMechanismLigament2d wristRootFix;
    private final LoggedMechanismLigament2d wrist;
    private final Superstructure superstructure;

    public SuperstructureVisualizer(Superstructure superstructure){
        this.superstructure = superstructure;
        // Create the canvas for mechanisms
        // Width is side frame perimeter (30 inches = 0.762 m) plus some buffer for the grabber, height is max elevator height + some buffer
        superstructureVis = new LoggedMechanism2d(1.35, 3);

        // Root for the elevator; 0.381 is center of frame
        elevatorRoot = superstructureVis.getRoot("Elevator Root", 0.381 + 0.191, 0.23114);
        //The elevator
        elevator = elevatorRoot.append(
            new LoggedMechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit("#770085"))
        );
        // The fixed part on the elevator that goes down for the grabber
        fixOnElevator = elevator.append(
            new LoggedMechanismLigament2d("Fix On Elevator", 0.271, -120, 10, new Color8Bit("#f2f2f2"))
        );
        // The grabber
        grabber = fixOnElevator.append(
            new LoggedMechanismLigament2d("Grabber", 0.4, 30, 10, new Color8Bit("#f2f2f2"))
        );

        // The root for the wrist; 0.381 is center of frame
        wristRoot = superstructureVis.getRoot("Wrist Root",  0.381 - 0.127, 0.23114);
        // The part that raises the wrist off the ground
        wristRootFix = wristRoot.append(
            new LoggedMechanismLigament2d("Fix for Wrist", 0.408, 90, 10, new Color8Bit("#f2f2f2"))
        );
        // The wrist itself
        wrist = wristRootFix.append(
            new LoggedMechanismLigament2d("Wrist", 0.3, 90, 10, new Color8Bit("#770085"))
        );
        SmartDashboard.putData("Superstructure Visualization", superstructureVis);
        Logger.recordOutput("Superstructure Visualization", superstructureVis);
        CommandUtils.makePeriodic(this::updateVisualization, true);
    }

    public LoggedMechanismLigament2d getElevatorLigament(){
        return elevator;
    }

    public LoggedMechanismLigament2d getWristLigament(){
        return wrist;
    }

    private void updateVisualization() {
        elevator.setLength(superstructure.getElevator().getPositionMeters() + 0.01); //can't set to min value or else advantage scope visualization disappears
        wrist.setAngle(superstructure.getWrist().getAngle());
    }

}