package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants.Control;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOHardware implements ShooterIO {
    private static final ShooterControlConstants realControlConstants = new ShooterControlConstants(
        Control.kP, Control.kI, Control.kD,
        Control.kS, Control.kV, Control.kA);

    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    private final Servo m_indexer;

    public ShooterIOHardware() {
        m_motor = new SparkMax(ShooterConstants.kMotorPort, MotorType.kBrushless);
        m_encoder = m_motor.getEncoder();
        m_indexer = new Servo(ShooterConstants.kServoPort);
    }
    
    @Override
    public void setVoltage(double Volts) {
        m_motor.setVoltage(Volts);
    }

    @Override
    public void setServoAngleDeg(double radians) {
        m_indexer.set(radians / 2*Math.PI);
    }

    @Override
    public ShooterControlConstants getShooterControlConstants() {
        return realControlConstants;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorCurrent = m_motor.getOutputCurrent();
        inputs.motorVelocityRadiansPerSecond = m_encoder.getVelocity() * 0.1047; // Convert from RPM to rad/s

        inputs.servoAngle = m_indexer.getPosition() * 2 * Math.PI;
    }
}