// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems.yoinker;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants.YoinkerConstants;
import frc.robot.Constants.YoinkerSimConstants;
import static frc.robot.Constants.YoinkerSimConstants.Control.*;

public class YoinkerIOSim implements YoinkerIO {

    private static final YoinkerControlConstants simControlConstants = new YoinkerControlConstants(
        kP, kI, kD, kProfileConstraints,
        kS, kG, kV, kA
    );

    private final DCMotor m_yoinkerGearbox;

    private SparkMax m_leadMotor;

    private final SparkMaxSim m_leadMotorSim;

    private SparkMaxConfig m_leadMotorConfig;

    private final SparkRelativeEncoderSim m_encoderSim;

    private final SingleJointedArmSim m_yoinkerSim;

    public YoinkerIOSim() {
        m_yoinkerGearbox = DCMotor.getNEO(2);

        m_leadMotor = new SparkMax(YoinkerConstants.kLeadMotorPort, MotorType.kBrushless);

        m_leadMotorConfig = new SparkMaxConfig();

        // create motor configurations
        m_leadMotorConfig
            .inverted(YoinkerConstants.kLeadMotorInverted)
            .idleMode(IdleMode.kBrake);

        m_leadMotorConfig.encoder
            .positionConversionFactor(YoinkerConstants.kPositionConversionFactor)
            .velocityConversionFactor(YoinkerConstants.kVelocityConversionFactor);

        // configure motors
        m_leadMotor.configure(m_leadMotorConfig, null, null);

        m_yoinkerSim = new SingleJointedArmSim(
            m_yoinkerGearbox,
            YoinkerSimConstants.kYoinkerGearingReductions,
            SingleJointedArmSim.estimateMOI(YoinkerSimConstants.kYoinkerLength, YoinkerSimConstants.kYoinkerMass),
            YoinkerSimConstants.kYoinkerLength,
            YoinkerSimConstants.kMinAngleRads,
            YoinkerSimConstants.kMaxAngleRads,
            true,
            0,
            //YoinkerSimConstants.kYoinkerEncoderDistPerPulse,
            0.0,
            0.0
        );

        m_leadMotorSim = new SparkMaxSim(m_leadMotor, m_yoinkerGearbox);
        m_encoderSim = m_leadMotorSim.getRelativeEncoderSim();

    }

    @Override
    public void simulationPeriodic() {
        m_yoinkerSim.setInput(m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage());

        m_yoinkerSim.update(0.020);

        m_encoderSim.setPosition(m_yoinkerSim.getAngleRads());
    }

    @Override
    public void setVoltage(double voltage) {
        m_leadMotorSim.setAppliedOutput(voltage/m_leadMotorSim.getBusVoltage());
    }

    @Override
    public YoinkerControlConstants getControlConstants() {
        return simControlConstants;
    }

    @Override
    public void updateInputs(YoinkerIOInputs inputs) {
        inputs.appliedVolts = m_leadMotorSim.getAppliedOutput() * m_leadMotorSim.getBusVoltage();
        inputs.leadCurrentAmps = m_leadMotorSim.getMotorCurrent();
        inputs.angleRadians = m_encoderSim.getPosition();
        inputs.velocityRadiansPerSecond = m_encoderSim.getVelocity();
    }
}