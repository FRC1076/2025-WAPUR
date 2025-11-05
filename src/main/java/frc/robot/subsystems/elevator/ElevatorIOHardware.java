package frc.robot.subsystems.elevator;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;
import static frc.robot.Constants.ElevatorConstants.Control.kProfileConstraints;


import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOHardware implements ElevatorIO 
{
    private static final ElevatorControlConstants realControlConstants = new ElevatorControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );
    
    private final SparkMax m_leadMotor; 
    private final SparkMax m_followMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkMaxConfig m_followMotorConfig;
    
    private final RelativeEncoder m_encoder;

    public ElevatorIOHardware() 
    {
        m_leadMotor = new SparkMax(ElevatorConstants.kMotorPort0, MotorType.kBrushless);
        m_followMotor = new SparkMax(ElevatorConstants.kMotorPort1, MotorType.kBrushless)
    }

    @Override
    public void setVoltage(double volts)
    {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public ElevatorControlConstants getControlConstants()
    {
        return realControlConstants;
    }

    @Override
    public void resetPosition(double positionMeters)
    {
        m_encoder.setPosition(positionMeters);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs)
    {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.appliedOutput = m_leadMotor.getAppliedOutput();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();

        inputs.followCurrentAmps = m_followMotor.getOutputCurrent();

        inputs.elevatorHeightMeters = m_encoder.getPosition();
        inputs.velocityMetersPerSecond = m_encoder.getVelocity();
    }
}