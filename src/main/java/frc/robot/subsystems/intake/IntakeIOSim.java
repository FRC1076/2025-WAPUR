package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
    private final DCMotor m_gearbox = DCMotor.getNEO(1);

    private SparkMax m_motor;
    private SparkMaxSim m_motorSim;

    public IntakeIOSim() {
        m_motor = new SparkMax(IntakeConstants.kMotorPort, MotorType.kBrushless);
        m_motorSim = new SparkMaxSim(m_motor, m_gearbox);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage();
        inputs.currentAmps = m_motorSim.getMotorCurrent();
    }
}
