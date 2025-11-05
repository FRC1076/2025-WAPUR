package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOHardware implements ShooterIO
{
    private final SparkMax m_motor;
    private final Servo m_indexer;

    public ShooterIOHardware() {
        m_motor = new SparkMax(ShooterConstants.kMotorPort, MotorType.kBrushless);
        m_indexer = new Servo(ShooterConstants.kServoPort);
    }
    
    @Override
    public void runVolts(double Volts) {
        m_motor.setVoltage(Volts);
    }

    @Override
    public void setServoAngleDeg(double radians) {
        m_indexer.set(radians / 2*Math.PI);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorCurrent = m_motor.getOutputCurrent();
    }
}