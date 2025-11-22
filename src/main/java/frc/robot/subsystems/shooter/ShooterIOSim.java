package frc.robot.subsystems.shooter;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.ControlSim;

public class ShooterIOSim implements ShooterIO {

    private final DCMotor m_gearbox = DCMotor.getNEO(1);

    private final SparkMax m_motor;
    private final SparkRelativeEncoderSim m_encoderSim;
    private final SparkMaxSim m_motorSim;

    private double indexPosition = 0;

    public ShooterIOSim() {
        m_motor = new SparkMax(ShooterConstants.kMotorPort, MotorType.kBrushless);
        m_motorSim = new SparkMaxSim(m_motor, m_gearbox);
        m_encoderSim = m_motorSim.getRelativeEncoderSim();
    }
    
    @Override
    public void setVelocityRadPerSec(double velocity) {
        if (velocity != 0) {
            setVoltage(12);
        } else {
            setVoltage(0);
        }
    }

    @Override
    public void setVoltage(double Volts) {
        m_motor.setVoltage(Volts);
    }

    @Override
    public void setServoAngleRad(double radians) {
        indexPosition = radians;
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.motorAppliedVoltage = m_motorSim.getAppliedOutput() * m_motorSim.getBusVoltage();
        inputs.motorCurrent = m_motorSim.getMotorCurrent();
        inputs.motorVelocityRadiansPerSecond = m_encoderSim.getVelocity() * 0.1047; // Convert from RPM to rad/s

        inputs.servoAngle = indexPosition;
    }
}
