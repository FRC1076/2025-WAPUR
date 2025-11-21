package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.Control;

public class ShooterIOTalon implements ShooterIO {
    private final TalonFX m_motor;
    private final TalonFXConfiguration m_motorConfig;

    // Velocity control
    private final MotionMagicVelocityVoltage m_velocityRequest = new MotionMagicVelocityVoltage(0);

    // Status Signals
    private final StatusSignal<Voltage> m_voltageSignal;
    private final StatusSignal<AngularVelocity> m_velocitySignal;
    private final StatusSignal<Current> m_currentSignal;

    private final Servo m_servo;

    public ShooterIOTalon(ShooterIO io) {
        m_motor = new TalonFX(ShooterConstants.kMotorPort);
        m_servo = new Servo(ShooterConstants.kServoPort);

        m_motorConfig = new TalonFXConfiguration();

        // Voltage and current configs
        m_motorConfig.Voltage.PeakForwardVoltage = 12;
        m_motorConfig.Voltage.PeakReverseVoltage = -12;
        m_motorConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.kCurrentLimit;

        // Inverted?
        m_motorConfig.MotorOutput.Inverted = ShooterConstants.kInverted;

        // Set brake mode
        m_motorConfig.MotorOutput.NeutralMode = ShooterConstants.kNeutralMode;

        // Closed loop
        m_motorConfig.Slot0.kP = Control.kP;
        m_motorConfig.Slot0.kI = Control.kI;
        m_motorConfig.Slot0.kD = Control.kD;
        m_motorConfig.Slot0.kS = Control.kS;  
        m_motorConfig.Slot0.kV = Control.kV;
        m_motorConfig.Slot0.kA = Control.kA;

        m_motorConfig.MotionMagic.MotionMagicAcceleration = Control.kMaxAcceleration;
        m_motorConfig.MotionMagic.MotionMagicJerk = Control.kMaxJerk; 

        // Just for fun
        m_motorConfig.Audio.AllowMusicDurDisable = true; 

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
        m_velocityRequest.Velocity = velocity / (2*Math.PI);
        m_motor.setControl(m_velocityRequest);
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
        inputs.motorVelocityRadiansPerSecond = m_velocitySignal.getValueAsDouble() * (2*Math.PI);
        inputs.motorCurrent = m_currentSignal.getValueAsDouble();

        inputs.servoAngle = m_servo.getAngle() * 2 * Math.PI;
    }
}
