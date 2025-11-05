package frc.robot.subsystems.grabber;

import frc.robot.Constants.GrabberConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GrabberIOHardware implements GrabberIO {
    private final SparkMax m_leftMotor;
    private final SparkMax m_rightMotor;

    private final RelativeEncoder m_encoder;

    private final SparkMaxConfig m_leftMotorConfig;
    private final SparkMaxConfig m_rightMotorConfig;

    public GrabberIOHardware() {
        m_leftMotor = new SparkMax(GrabberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_rightMotor = new SparkMax(GrabberConstants.kRightMotorPort, MotorType.kBrushless);

        m_encoder = m_leftMotor.getEncoder();

        m_leftMotorConfig = new SparkMaxConfig();
        m_rightMotorConfig = new SparkMaxConfig();

        m_leftMotorConfig
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kLeftMotorInverted)
        .encoder
            .positionConversionFactor(GrabberConstants.kPositionConversionFactor);
            
        m_rightMotorConfig
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kRightMotorInverted);

        m_leftMotor.configure(m_leftMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_rightMotor.configure(m_rightMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void runVolts(double volts) {
        m_leftMotor.setVoltage(volts);
        m_rightMotor.setVoltage(volts);
    }

    @Override
    public void runVoltsDifferential(double leftMotorVolts, double rightMotorVolts) {
        m_leftMotor.setVoltage(leftMotorVolts);
        m_rightMotor.setVoltage(rightMotorVolts);
    }

    @Override
    public double getOutputCurrent() {
        return (m_leftMotor.getOutputCurrent() + m_rightMotor.getOutputCurrent()) / 2;
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.leftMotorAppliedVoltage = m_leftMotor.getAppliedOutput() * m_leftMotor.getBusVoltage();
        inputs.leftMotorCurrent = m_leftMotor.getOutputCurrent();
        inputs.leftMotorRPM = m_leftMotor.getEncoder().getVelocity();
        
        inputs.rightMotorAppliedVoltage = m_rightMotor.getAppliedOutput() * m_rightMotor.getBusVoltage();
        inputs.rightMotorCurrent = m_rightMotor.getOutputCurrent();
        inputs.rightMotorRPM = m_rightMotor.getEncoder().getVelocity();

        inputs.motorPositionRadians = m_encoder.getPosition();
    }
}