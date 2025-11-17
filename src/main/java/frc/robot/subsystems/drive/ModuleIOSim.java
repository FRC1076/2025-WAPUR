// Copyright (c) 2021-2025 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Drive;
import frc.robot.Constants.DriveConstants.ModuleConstants.Common.Turn;
import frc.robot.Constants.DriveConstants.SimConstants;

/** Physics sim implementation of module IO. */
public class ModuleIOSim implements ModuleIO {
    private final DCMotorSim driveSim;
    private final DCMotorSim turnSim;

    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private PIDController driveController = new PIDController(SimConstants.kDriveKP, 0, SimConstants.kDriveKD);
    private PIDController turnController = new PIDController(SimConstants.kTurnKP, 0, SimConstants.kTurnKD);
    private double driveFFVolts = 0.0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    private final ConcurrentLinkedQueue<Long> timestampQueue;
    private final ConcurrentLinkedQueue<Double> drivePositionQueue;
    private final ConcurrentLinkedQueue<Double> turnPositionQueue; 

    private final ArrayList<Long> timestampBuffer = new ArrayList<>();
    private final ArrayList<Double> drivePositionBuffer = new ArrayList<>();
    private final ArrayList<Double> turnPositionBuffer = new ArrayList<>();

    public ModuleIOSim() {
        // Create drive and turn sim models
        driveSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.025, Drive.gearRatio),
                DCMotor.getNEO(1));
        turnSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), 0.004, Turn.gearRatio),
                DCMotor.getNEO(1));

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        timestampQueue = OdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = OdometryThread.getInstance().registerSignal(() -> driveSim.getAngularPositionRad() * 0.1016);
        turnPositionQueue = OdometryThread.getInstance().registerSignal(() -> turnSim.getAngularPositionRad());
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts + driveController.calculate(driveSim.getAngularVelocityRadPerSec());
        } else {
            driveController.reset();
        }

        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(turnSim.getAngularPositionRad());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveSim.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        turnSim.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        driveSim.update(0.02);
        turnSim.update(0.02);

        // Update drive inputs
        inputs.driveVelocityMetersPerSec = driveSim.getAngularVelocityRadPerSec() * 0.1016; // 0.1016 converts from radians to meters
        inputs.driveAppliedVolts = driveAppliedVolts;
        inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.turnPosition = turnSim.getAngularPositionRad();
        inputs.turnVelocityRadiansPerSecond = turnSim.getAngularVelocityRadPerSec();
        inputs.turnAppliedVolts = turnAppliedVolts;
        inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

        int samples = OdometryThread.getInstance().sampleCount;
        OdometryThread.safeDrain(timestampQueue, timestampBuffer,samples);
        OdometryThread.safeDrain(drivePositionQueue, drivePositionBuffer, samples);
        OdometryThread.safeDrain(turnPositionQueue, turnPositionBuffer, samples);
        inputs.odometryTimestamps = timestampBuffer.stream().mapToDouble((Long value) -> value/1e6).toArray();
        inputs.odometryDrivePositionsMeters = drivePositionBuffer.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions = turnPositionBuffer.stream().map((Double value) -> Rotation2d.fromRadians(value)).toArray(Rotation2d[]::new);
        timestampBuffer.clear();
        drivePositionBuffer.clear();
        turnPositionBuffer.clear();
    }

    @Override
    public void setDriveVolts(double volts) {
        driveClosedLoop = false;
        driveAppliedVolts = volts;
    }

    @Override
    public void setTurnVolts(double volts) {
        turnClosedLoop = false;
        turnAppliedVolts = volts;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec, double FFVolts) {
        driveClosedLoop = true;
        driveFFVolts = FFVolts;
        driveController.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(double angleRadians, double FFVolts) {
        turnClosedLoop = true;
        turnController.setSetpoint(angleRadians);
    }
}
