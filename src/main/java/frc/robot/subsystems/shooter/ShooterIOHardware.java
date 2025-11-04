package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOHardware implements ShooterIO
{
    private final CANSparkMax m_Motor;
    private final Servo m_Indexer;

    public ShooterIOHardware() 
    {
        m_Motor = new CANSparkMax(ShooterConstants.kMotorPort, CANSparkMax.MotorType.kBrushless);
        m_Indexer = new Servo(ShooterConstants.kServoPort);
    }
    
    @Override
    public void runVolts(double Volts) 
    {
        m_Motor.setVoltage(Volts);
    }

    @Override
    public void setServoAngleDeg(double degrees) 
    {
        m_Indexer.set(degrees / 180);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) 
    {
        inputs.motorAppliedVoltage = m_Motor.getAppliedOutput() * m_Motor.getBusVoltage();
        inputs.motorCurrent = m_Motor.getOutputCurrent();
    }
}