// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.grabber;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.GrabberConstants;

/** 
 * A simple boilerplate class to enable full superstructure simulation
 */
public class GrabberIOSim implements GrabberIO {
    private DCMotor m_gearbox = DCMotor.getNEO(2);

    private final SparkMax m_leadMotor;
    private final SparkMax m_followMotor;

    private final SparkMaxConfig m_leadMotorConfig;
    private final SparkMaxConfig m_followMotorConfig;

    private final SparkMaxSim m_leadMotorSim;
    private final SparkMaxSim m_followMotorSim;

    private final SparkRelativeEncoderSim m_encoderSim;

    public GrabberIOSim() {
        m_leadMotor = new SparkMax(GrabberConstants.kLeftMotorPort, MotorType.kBrushless);
        m_followMotor = new SparkMax(GrabberConstants.kRightMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();
        m_followMotorConfig = new SparkMaxConfig();

        m_leadMotorConfig
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kLeftMotorInverted)
        .encoder
            .positionConversionFactor(GrabberConstants.kPositionConversionFactor)
            .velocityConversionFactor(GrabberConstants.kVelocityConversionFactor);
            
        m_followMotorConfig
            .follow(m_leadMotor)
            .smartCurrentLimit((int) GrabberConstants.kCurrentLimit)
            .inverted(GrabberConstants.kRightMotorInverted != GrabberConstants.kLeftMotorInverted);

        m_leadMotor.configure(m_leadMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        m_followMotor.configure(m_followMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_gearbox);
        m_followMotorSim = new SparkMaxSim(m_followMotor, m_gearbox);

        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();
    }

    @Override
    public void setVoltage(double volts) {
        m_leadMotor.setVoltage(volts);
    }

    @Override
    public void updateInputs(GrabberIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadMotorCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.followMotorCurrentAmps = m_followMotorSim.getMotorCurrent();
        
        inputs.motorPositionRadians = m_encoderSim.getPosition();
        inputs.velocityRadPerSec = m_encoderSim.getVelocity();
    }

    @Override
    public void simulationPeriodic() {
        // nothing I geuess
    }    
}