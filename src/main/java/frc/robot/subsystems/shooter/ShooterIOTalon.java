package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.Control;
import frc.robot.utils.MusicUtil;
import lib.units.TalonFXUnitConverter;

public class ShooterIOTalon implements ShooterIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;
    private final TalonFXUnitConverter m_unitConverter;

    // Velocity control
    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);

    // Status Signals
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Current> m_currentSignal;

    private final Servo m_servo;

    public ShooterIOTalon() {
        m_motor = new TalonFX(ShooterConstants.kMotorPort);
        m_servo = new Servo(ShooterConstants.kServoPort);

        m_motorConfig = new TalonFXConfiguration();
        m_unitConverter = new TalonFXUnitConverter();

        // Voltage and current configs
        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kCurrentLimit;

        // Inverted?
        m_motorConfig.MotorOutput.Inverted = ShooterConstants.kInverted;

        // Set brake mode
        m_motorConfig.MotorOutput.NeutralMode = ShooterConstants.kNeutralMode;

        // Closed loop
        m_motorConfig.Slot0.kP = m_unitConverter.fromSIkP(Control.kP);
        m_motorConfig.Slot0.kI = m_unitConverter.fromSIkI(Control.kI);
        m_motorConfig.Slot0.kD = m_unitConverter.fromSIkD(Control.kD);
        m_motorConfig.Slot0.kS = m_unitConverter.fromSIkS(Control.kS);  
        m_motorConfig.Slot0.kV = m_unitConverter.fromSIkV(Control.kV);
        m_motorConfig.Slot0.kA = m_unitConverter.fromSIkA(Control.kA);

        m_motorConfig.MotionMagic.MotionMagicAcceleration = m_unitConverter.fromSIAccel(Control.kMaxAcceleration);
        m_motorConfig.MotionMagic.MotionMagicJerk = m_unitConverter.fromSIJerk(Control.kMaxJerk); 

        // Just for fun
        m_motorConfig.Audio.AllowMusicDurDisable = true; 
        MusicUtil.addInstrument(m_motor);

        // Actually configure the motor
        m_motor.getConfigurator().apply(m_motorConfig);

        // Set up status signals
        m_voltageSignal = m_motor.getMotorVoltage();
        m_velocitySignal = m_motor.getVelocity();
        m_currentSignal = m_motor.getTorqueCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        m_motor.setVoltage(volts);
    }

    @Override
    public void setVelocityRadPerSec(double velocity) {
        if (velocity != 0) {
            m_velocityRequest.Velocity = m_unitConverter.fromSIVel(velocity);
            m_motor.setControl(m_velocityRequest);
        } else { 
            m_motor.setVoltage(0);
        }
    }

    @Override
    public void setServoAngleRad(double radians) {
        m_servo.set(radians / (2*Math.PI));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        m_voltageSignal.refresh();
        m_velocitySignal.refresh();
        m_currentSignal.refresh();

        inputs.motorAppliedVoltage = m_voltageSignal.getValueAsDouble();
        inputs.motorVelocityRadiansPerSecond = m_unitConverter.toSIVel(m_velocitySignal.getValueAsDouble());
        inputs.motorCurrent = m_currentSignal.getValueAsDouble();

        inputs.servoAngle = m_servo.getAngle() * 2 * Math.PI;
    }
}
