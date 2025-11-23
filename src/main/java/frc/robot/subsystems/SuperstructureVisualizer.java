// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import lib.extendedcommands.CommandUtils;

/*
 * This class is used to create mechanism visualizations and pass them into IOSim objects
 */
public class SuperstructureVisualizer {
    
    private final Mechanism2d superstructureVis;

    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevator;
    private final MechanismLigament2d fixOnElevator;
    private final MechanismLigament2d grabber;

    private final MechanismRoot2d wristRoot;
    private final MechanismLigament2d wristRootFix;
    private final MechanismLigament2d wrist;
    private final Superstructure superstructure;

    public SuperstructureVisualizer(Superstructure superstructure){
        this.superstructure = superstructure;
        // Create the canvas for mechanisms
        // Width is side frame perimeter (30 inches = 0.762 m) plus some buffer for the grabber, height is max elevator height + some buffer
        superstructureVis = new Mechanism2d(1.35, 3);

        // Root for the elevator; 0.381 is center of frame
        elevatorRoot = superstructureVis.getRoot("Elevator Root", 0.381 + 0.191, 0.23114);
        //The elevator
        elevator = elevatorRoot.append(
            new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit("#770085"))
        );
        // The fixed part on the elevator that goes down for the grabber
        fixOnElevator = elevator.append(
            new MechanismLigament2d("Fix On Elevator", 0.271, -120, 10, new Color8Bit("#f2f2f2"))
        );
        // The grabber
        grabber = fixOnElevator.append(
            new MechanismLigament2d("Grabber", 0.4, 30, 10, new Color8Bit("#f2f2f2"))
        );

        // The root for the wrist; 0.381 is center of frame
        wristRoot = superstructureVis.getRoot("Wrist Root",  0.381 - 0.127, 0.23114);
        // The part that raises the wrist off the ground
        wristRootFix = wristRoot.append(
            new MechanismLigament2d("Fix for Wrist", 0.408, 90, 10, new Color8Bit("#f2f2f2"))
        );
        // The wrist itself
        wrist = wristRootFix.append(
            new MechanismLigament2d("Wrist", 0.3, 90, 10, new Color8Bit("#770085"))
        );
        SmartDashboard.putData("Superstructure Visualization", superstructureVis);
        CommandUtils.makePeriodic(this::updateVisualization, true);
    }

    public MechanismLigament2d getElevatorLigament(){
        return elevator;
    }

    public MechanismLigament2d getWristLigament(){
        return wrist;
    }

    private void updateVisualization() {
        elevator.setLength(superstructure.getElevator().getPositionMeters() + 0.01); //can't set to min value or else advantage scope visualization disappears
        wrist.setAngle(superstructure.getWrist().getAngle());
    }

}