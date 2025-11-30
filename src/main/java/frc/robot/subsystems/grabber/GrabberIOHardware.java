package frc.robot.subsystems.grabber;

import frc.robot.Constants.GrabberConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

public class GrabberIOHardware implements GrabberIO {
    private final SparkMax m_leadMotor;
    private final SparkMax m_followMotor;

    private final RelativeEncoder m_encoder;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkMaxConfig m_followMotorConfig;

    public GrabberIOHardware() {
        m_leadMotor = new SparkMax(GrabberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_followMotor = new SparkMax(GrabberConstants.kRightMotorPort, MotorType.kBrushless);

        m_encoder = m_leadMotor.getEncoder();

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        m_leadMotorConfig
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kLeftMotorInverted)
        .encoder
            .positionConversionFactor(GrabberConstants.kPositionConversionFactor)
            .velocityConversionFactor(GrabberConstants.kVelocityConversionFactor);
            
        m_followMotorConfig
            .follow(m_leadMotor, GrabberConstants.kRightMotorInverted != GrabberConstants.kLeftMotorInverted)
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public double getOutputCurrent() {
        return (m_leadMotor.getOutputCurrent() + m_followMotor.getOutputCurrent()) / 2;
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadMotorCurrentAmps = m_leadMotor.getOutputCurrent();
        
        inputs.followMotorCurrentAmps = m_followMotor.getOutputCurrent();

        inputs.motorPositionRadians = m_encoder.getPosition();
        inputs.velocityRadPerSec = m_encoder.getVelocity();
    }
}