package frc.robot.subsystems.grabber;

import frc.robot.commands.grabber.ApplyRadians;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ElevatorConstants.Electrical.kCurrentLimit;

import org.littletonrobotics.junction.Logger;

public class GrabberSubsystem extends SubsystemBase
{
    private final GrabberIO io;
    private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();
    private final LinearFilter currentFilter;
    private final double kCoralCurrentThreshold;
    private final double kCoralFunnelIntakeThreshold;
    private double filteredCurrent;

    public GrabberSubsystem(GrabberIO io) 
    {
        this.io = io;
        this.currentFilter = LinearFilter.movingAverage(8);
        this.kCoralCurrentThreshold = 12;
        this.kCoralFunnelIntakeThreshold = 5;
    }

    public void runVolts(double volts) 
    {
        this.io.runVolts(volts);
    }

    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) 
    {
        this.io.runVoltsDifferential(leftMotorVolts, rightMotorVolts);
    }

    public void stop() 
    {
        runVolts(0);
    }

    public double getPosition() 
    {
        return inputs.motorPositionRadians;
    }

    public boolean hasCoral() 
    {
        return filteredCurrent > kCoralCurrentThreshold;
    }

    public boolean hasFunnelCurrentSpike()
    {
        return getAppliedCurrent() > kCoralFunnelIntakeThreshold;
    }

    @Override
    public void periodic() {
        filteredCurrent = currentFilter.calculate(io.getOutputCurrent() > 25 ? 0 : io.getOutputCurrent());
        io.updateInputs(inputs);
        Logger.processInputs("Grabber", inputs);
    }

    public Command applyVoltage(double volts) 
    {
        return runOnce(() -> runVolts(volts));
    }
    
    public Command applyDifferentialVolts(double leftMotorVolts, double rightMotorVolts) 
    {
        return runOnce(() -> runVoltsDifferential(leftMotorVolts, rightMotorVolts));
    }

    public double getAppliedCurrent() 
    {
        return inputs.leftMotorCurrent;
    }

    public Command applyRadiansBangBang(double volts, double radians) 
    {
        return new ApplyRadians(volts, radians, this);
    }

    public Command applyRotationsBangBang(double volts, double rotations) 
    {
        return applyRadiansBangBang(volts, rotations * 2 * Math.PI);
    }
}