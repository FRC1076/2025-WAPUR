package frc.robot.subsystems.elevator;

import frc.robot.Constants.ElevatorConstants;

import static frc.robot.Constants.ElevatorConstants.Control.kA;
import static frc.robot.Constants.ElevatorConstants.Control.kD;
import static frc.robot.Constants.ElevatorConstants.Control.kI;
import static frc.robot.Constants.ElevatorConstants.Control.kP;
import static frc.robot.Constants.ElevatorConstants.Control.kS;
import static frc.robot.Constants.ElevatorConstants.Control.kV;
import static frc.robot.Constants.ElevatorConstants.Control.kG;
import static frc.robot.Constants.ElevatorConstants.Control.kProfileConstraints;

import static frc.robot.Constants.ElevatorConstants.kMotorPort0;
import static frc.robot.Constants.ElevatorConstants.kMotorPort1;
import static frc.robot.Constants.ElevatorConstants.kPositionConversionFactor;
import static frc.robot.Constants.ElevatorConstants.kVelocityConversionFactor;
import static frc.robot.Constants.ElevatorConstants.Electrical.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

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