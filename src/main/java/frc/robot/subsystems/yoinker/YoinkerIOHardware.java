package frc.robot.subsystems.yoinker;

import frc.robot.Constants.YoinkerConstants;

import static frc.robot.Constants.YoinkerConstants.Control.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class YoinkerIOHardware implements YoinkerIO{
    private static final YoinkerControlConstants realControlConstants = new YoinkerControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );

    private final SparkMax m_leadMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkAbsoluteEncoder m_absoluteEncoder;

    public YoinkerIOHardware() {
        m_leadMotor = new SparkMax(YoinkerConstants.kLeadMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        
        m_leadMotorConfig
            .inverted(YoinkerConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit((int) YoinkerConstants.kSmartCurrentLimit);
 

        m_leadMotorConfig.absoluteEncoder
            .setSparkMaxDataPortConfig()
            .inverted(true)
            .positionConversionFactor(YoinkerConstants.kPositionConversionFactor)
            .velocityConversionFactor(YoinkerConstants.kVelocityConversionFactor);
            
        m_leadMotorConfig.encoder
            .positionConversionFactor(YoinkerConstants.kPositionConversionFactor)
            .velocityConversionFactor(YoinkerConstants.kVelocityConversionFactor);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_absoluteEncoder = m_leadMotor.getAbsoluteEncoder();

    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public YoinkerControlConstants getControlConstants() {
        return realControlConstants;
    }

    @Override
    public void updateInputs(YoinkerIOInputs inputs) {
        inputs.appliedVolts = m_leadMotor.getAppliedOutput() * m_leadMotor.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotor.getOutputCurrent();
        inputs.angleRadians = ((m_absoluteEncoder.getPosition()  - YoinkerConstants.kZeroOffsetRadians + Math.PI) % (2 * Math.PI) - Math.PI);
        inputs.velocityRadiansPerSecond = m_absoluteEncoder.getVelocity();
    }
}
