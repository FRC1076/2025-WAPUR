package frc.robot.subsystems.shooter;

import frc.robot.Constants.ShooterConstants.Control;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSpark implements ShooterIO {

    private final SparkMax m_motor;
    private final SparkMaxConfig m_motorConfig;
    private final RelativeEncoder m_encoder;
    private final SparkClosedLoopController m_closedLoopController;
    private final Servo m_indexer;

    public ShooterIOSpark() {
        m_motor = new SparkMax(ShooterConstants.kMotorPort, MotorType.kBrushless);

        m_motorConfig = new SparkMaxConfig();
            
        m_motorConfig
            .smartCurrentLimit(40)
            .voltageCompensation(12);

        m_motorConfig.closedLoop
            .p(Control.kP)
            .i(Control.kI)
            .d(Control.kD)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        m_closedLoopController = m_motor.getClosedLoopController();
        m_encoder = m_motor.getEncoder();
        m_indexer = new Servo(ShooterConstants.kServoPort);
    }
    
    @Override
    public void setVoltage(double Volts) {
        m_motor.setVoltage(Volts);
    }

    @Override
    public void setVelocityRadPerSec(double velocity) {
        m_closedLoopController.setReference(
            velocity, 
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            Control.kS,
            ArbFFUnits.kVoltage);
    }

    @Override
    public void setServoAngleDeg(double radians) {
        m_indexer.set(radians / 2*Math.PI);
    }
    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = m_motor.getAppliedOutput() * m_motor.getBusVoltage();
        inputs.motorCurrent = m_motor.getOutputCurrent();
        inputs.motorVelocityRadiansPerSecond = m_encoder.getVelocity() * 0.1047; // Convert from RPM to rad/s

        inputs.servoAngle = m_indexer.getPosition() * 2 * Math.PI;
    }
}