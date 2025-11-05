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
import com.revrobotics.spark.SparkBase.PersistMode;
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

    public ElevatorIOHardware() /*need edit*/
    {
        m_leadMotor = new SparkMax(ElevatorConstants.kMotorPort0, SparkMax.MotorType.kBrushless);
        m_followMotor = new SparkMax(ElevatorConstants.kMotorPort1, SparkMax.MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        m_leadMotor.setCANTimeout(250);
        m_followMotor.setCANTimeout(250);

        m_leadMotorConfig
            .inverted(ElevatorConstants.leadMotorInverted)
            .smartCurrentLimit((int) ElevatorConstants.Electrical.kCurrentLimit)
            .voltageCompensation(ElevatorConstants.Electrical.kVoltageCompensation);

        m_leadMotorConfig.encoder
            .positionConversionFactor(ElevatorConstants.kPositionConversionFactor)
            .velocityConversionFactor(ElevatorConstants.kVelocityConversionFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);

        m_followMotorConfig
            .smartCurrentLimit((int) ElevatorConstants.Electrical.kCurrentLimit)
            .voltageCompensation(ElevatorConstants.Electrical.kVoltageCompensation)
            .follow(m_leadMotor, ElevatorConstants.followMotorInverted != ElevatorConstants.leadMotorInverted);
        
        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        m_encoder = m_leadMotor.getEncoder();

        m_encoder.setPosition(0);

        m_leadMotor.setCANTimeout(0);
        m_followMotor.setCANTimeout(0);
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