package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.IntakeConstants;

public class IntakeIOHardware implements IntakeIO {
    private SparkMax m_motor;

    public IntakeIOHardware() {
        m_motor = new SparkMax(IntakeConstants.kMotorPort, MotorType.kBrushless);
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.appliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.currentAmps = m_motor.getOutputCurrent();
    }
}
