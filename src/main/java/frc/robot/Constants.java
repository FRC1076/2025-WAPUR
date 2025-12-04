package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public final class Constants {  
    public static class GameConstants {
        public static Alliance teamColor = Alliance.Blue;
        public static AutonSides autonSide = AutonSides.Left;
        public static boolean rearRightCameraEnabledAuton = false;
        
        public enum TeamColors 
        {
            kTeamColorBlue("BLUE"),
            kTeamColorRed("RED");

            public final String color;

            private TeamColors(String color) {
                this.color = color;
            }
        }
        
        public enum AutonSides {
            Left(false),
            Right(true);

            public final boolean isRightSide;

            private AutonSides (boolean isRightSide) {
                this.isRightSide = isRightSide;
            }
        }
    }

    public static class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;

        public static final RumbleType kRumbleType = RumbleType.kBothRumble;
        public static final double kRumbleIntensity = 1;

        public static final String kDriverCameraIP = "10.10.76.11";
        public static final int kDriverCameraPort = 5801;

        public static final OperatorControllerStates kOperatorControllerState = OperatorControllerStates.SOUNDS;

        public static enum OperatorControllerStates {
            SOUNDS,
            NOTHING,
            DRIVETRAIN_SYSID_TRANS,
            DRIVETRAIN_SYSID_SPIN,
            ELEVATOR_SYSID,
            WRIST_SYSID,
            SHOOTER_SYSID;
        }
    }

    public static class SystemConstants {
        public static final RobotMode currentMode = RobotMode.REAL;
        
        public static final boolean signalLoggerEnabled = false;
        public static final boolean increaseThreadPriority = true;

        public static enum RobotMode {
            REAL(true),
            REAL_NO_MECHANISMS(true),
            REAL_NO_DRIVETRAIN(true),
            SIM(false),
            REPLAY(false);

            public final boolean isReal;

            private RobotMode(boolean isReal) {
                this.isReal = isReal;
            }
        }
    }

    public static class ElevatorConstants  {
        public static final int kMotorPort0 = 31;
        public static final int kMotorPort1 = 32;
        
        public static final double elevatorPositionToleranceMeters = Units.inchesToMeters(1);
        public static final double kMinElevatorHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(69.95);
        
        public static final double defaultMaxOperatorControlVolts = 1.5;
        public static final double fasterMaxOperatorControlVolts = 4;

        public static final boolean leadMotorInverted = false;
        public static final boolean followMotorInverted = false;

        public static final double kGearRatio = 10.909; // TODO: Confirm is this is double the 60:11 ratio because of the two chains
        public static final double kElevatorStages = 3;
        public static final double kSprocketToothCount = 22;
        public static final double kSprocketPitch = Units.inchesToMeters(1.0/4); // Pitch is the distance between two adjacent teeth
        public static final double kPositionConversionFactor = ((kSprocketToothCount * kSprocketPitch) / kGearRatio) * kElevatorStages; // TODO: Confirm conversion factors
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0; // 60 converts from minutes to seconds
        
        /* New factor math
         * (sprocket pitch circumference / gear ratio) * stage count
         * sprocket pitch circumference = pitch * tooth count
         * 
         * Original conversion factor math
         * 0.00635 m = 1/4 inch, which is the pitch
         * 24/22 = wrong; should be 22/24; meaning a 22t sprocket drives a 24t sprocket
         * 24 = 24t drive sprocket; should cancel out with 22/24
         * 
         * (24.0/22.0) * kElevatorStages * (1/kGearRatio) * 24 * 0.00635
         */

        public static class Electrical {
            public static final double kVoltageCompensation = 10.5; 
            public static final double kCurrentLimit = 60;
        }

        public static class Control {
            public static final double kP = 30;
            public static final double kI = 0.0;
            public static final double kD = 0;

            public static final double kS = 0.17213;
            public static final double kG = 0.77965;
            public static final double kV = 3.0126;
            public static final double kA = 0.73282;

            public static final Constraints kProfileConstraints = new Constraints(3, 3);
        }
    }

    public static class ElevatorSimConstants {
        public static final int kSimMotorPort0 = 20;
        public static final int kSimMotorPort1 = 21;
        
        public static final double kElevatorGearing = 60.0/11.0;
        public static final double kCarriageMass = Units.lbsToKilograms(30);
        public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
        public static final double kMinElevatorHeightMeters = 0.0;
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(72.0);

        public static final int kEncoderAChannel = 0;
        public static final int kEncoderBChannel = 1;

        public static class Control 
        {
            public static final double kP = 6.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0;
            public static final double kG = 2.8605;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(1, 3);
        }
    }

    public static final class GrabberConstants {
        public static final int kLeftMotorPort = 38;
        public static final int kRightMotorPort = 39;
        
        public static final double kCurrentLimit = 40; 
        public static final double kGearRatio = 12;
        public static final double kPositionConversionFactor = Math.PI * 2 * (1/kGearRatio);
        public static final double kVelocityConversionFactor = kPositionConversionFactor / 60.0;

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;

        public static final double kOperatorControlVolts = 6;

        public static final int kCurrentFilterTaps = 8;
        public static final double kIntakeCurrentSpike = 30;
        public static final double kIntakeCurrentSpikeDebounceSecs = 0.04;
        public static final double kEjectCurrentDrop = 6;
        public static final double kEjectCurrentDropDebounceSecs = 0.04;
    }

    public static class ShooterConstants {
        public static final int kMotorPort = 43;
        public static final int kServoPort = 9;

        public static final int kCurrentLimit = 40;

        public static final double kManualShootVolts = 12;
        public static final double kManualReverseVolts = 4;

        public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        public static final double kServoAngleUpRad = ((4*Math.PI) / 3) + (Math.PI/36); 
        public static final double kServoAngleDownRad = ((2*Math.PI) / 3) + (Math.PI/36); // TODO: check these angles, they are estimates

        public static class Control {
            // TODO: make these more aggressive, maybe?
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kV = 0.025;
            public static final double kA = 0;

            public static final double kMaxAcceleration = 20000;
            public static final double kMaxJerk = 160000;
        }

        public static class ControlSim {
            public static final double kP = 1;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kV = 0.02;
            public static final double kA = 0;
        }
    }

    public static class WristConstants {
        public static final int kLeadMotorPort = 41; // TODO: check based on unpushed code

        public static final double wristAngleToleranceRadians = 0.1;
        public static final double kMinWristAngleRadians = -Math.PI / 4;
        public static final double kMaxWristAngleRadians = Math.PI / 2;

        public static final double maxOperatorControlVolts = 1;
        public static final double kSmartCurrentLimit = 40.0;

        public static final boolean kLeadMotorInverted = false;

        public static final double kPositionConversionFactor = 2 * Math.PI;
        public static final double kVelocityConversionFactor = 2 * Math.PI / 60;
        public static final double kInitialPosition = Math.PI / 2;

        public static final class Control {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);
        }
    }

    public static class WristSimConstants {
        public static final double kWristGearingReductions = 12;
        public static final double kWristLength = 0.2;
        public static final double kWristMass = 2.0;
        public static final double kMinAngleRads = -0.75 * Math.PI;
        public static final double kMaxAngleRads = 0.75 * Math.PI;

        public static class Control 
        {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(2, 2);
        }


    }

    public static class IntakeConstants {
        public static final int kMotorPort = 42;

        public static final double kManualIntakeVolts = 4;
        public static final double kManualEjectVolts = 2;
    }

    public static class YoinkerConstants {
        public static final int kLeadMotorPort = 62;

        public static final double kYoinkerAngleToleranceRadians = 0.1;
        public static final double kMinYoinkerAngleRadians = 0;
        public static final double kMaxYoinkerAngleRadians = Math.PI / 2;

        public static final double maxOperatorControlVolts = 0;
        public static final double kSmartCurrentLimit = 40.0;

        public static final boolean kLeadMotorInverted = true;

        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2 * Math.PI;
        public static final double kVelocityConversionFactor = 2 * Math.PI / 60;
        public static final double kZeroOffsetRadians = 0;

        public static final class Control {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);
        }
    }

    public static class YoinkerSimConstants {
        public static final double kYoinkerGearingReductions = 64;
        public static final double kYoinkerLength = 0.2;
        public static final double kYoinkerMass = 2.0;
        public static final double kMinAngleRads = -0.75 * Math.PI;
        public static final double kMaxAngleRads = 0.75 * Math.PI;

        public static class Control 
        {
            public static final double kP = 0.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0;
            public static final double kG = 0.0;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(2, 2);
        }


    }

    public static class SuperstructureConstants {
        public static enum BallStates {
            HOME(Math.PI/2,300.0, 0.0, ShooterConstants.kServoAngleDownRad),
            INTAKE_DOWN(-Math.PI/4,300.0, 0.0,ShooterConstants.kServoAngleDownRad),
            INTAKING(-Math.PI/4,300.0, 4.0, ShooterConstants.kServoAngleDownRad),
            SHOOT(-Math.PI/4, 700.0, 0.0, ShooterConstants.kServoAngleUpRad),
            SHOOT_WRIST_UP(Math.PI/2, 700.0, 0.0, ShooterConstants.kServoAngleUpRad);

            public final double wristAngleRadians;
            public final double shooterRadPerSec;
            public final double intakeSpeed;
            public final double servoAngleRad;

            private BallStates (double wristAngleRadians, double shooterRadPerSec, double intakeSpeed, double servoAngleRad) {
                this.wristAngleRadians = wristAngleRadians;
                this.shooterRadPerSec = shooterRadPerSec;
                this.intakeSpeed = intakeSpeed;
                this.servoAngleRad = servoAngleRad;
            }
        }

        public static enum CrateStates {
            HOME(0.0,0.0),
            INTAKE_CRATES(0.0,-6.0),
            TRAVEL(0.025,0.0),
            PRE_L1(0.025,0.0),
            PRE_L2(0.415,0.0),
            PRE_L3(0.83,0.0),
            PRE_L4(1.21,0.0),
            SHOOT_L1(0.025,6.0),
            SHOOT_L2(0.415,6.0),
            SHOOT_L3(0.83,6.0),
            SHOOT_L4(1.21,6.0);

            public final double elevatorHeight; 
            public final double grabberVoltage;

            private CrateStates(double elevatorHeight, double grabberVoltage) {
                this.elevatorHeight = elevatorHeight;
                this.grabberVoltage = grabberVoltage;
            }

        }

    }

    public static class MusicConstants {
        public static final String kMusicPathXButton = "";
        public static final String kMusicPathYButton = "";
        public static final String kMusicPathBButton = "";
    }

    // TODO: check all the constants in this class
    public static class DriveConstants {
        public static final double maxTranslationSpeedMPS = Units.feetToMeters(5);
        public static final double maxRotationSpeedRadPerSec = 4; // originally set to 2 // Maximum acceptable value appears to be 12

        public static final boolean useSpeedScaling = true;

        public static final double singleClutchTranslationFactor = 0.6;
        public static final double singleClutchRotationFactor = 0.6;
        public static final double doubleClutchTranslationFactor = 0.35;
        public static final double doubleClutchRotationFactor = 0.35;

        public static final int odometryFrequencyHz = 100;
        public static final double wheelBase = Units.inchesToMeters(27.5);
        public static final double trackWidth = Units.inchesToMeters(27.5);
        //public static final double wheelRadius = 0.0508; //Meters

        public static final Translation2d[] moduleTranslations = new Translation2d[] {
            new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
            new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
        };

        public static final InterpolatingDoubleTreeMap elevatorAccelerationTable = new InterpolatingDoubleTreeMap(); // A table that maps elevator heights to slew rate limits
        static {
            elevatorAccelerationTable.put(0.0,100000.0);
            elevatorAccelerationTable.put(1.0,100000.0); // Deadzone with no acceleration limiting between 0.0 and 1.348 (THE END OF THIS DEADZONE *MUST* BE SLIGHTLY LOWER THAN THE POINT WHERE WE ACTUALLY WANT ELEVATOR ACCELERATION LIMITING TO BEGIN)
            // elevatorAccelerationTable.put(0.0, 12.66793578);
            // elevatorAccelerationTable.put(0.253, 100000.0);
            // elevatorAccelerationTable.put(0.254, 10.15773958 / 5);
            // elevatorAccelerationTable.put(0.508, 8.477828029 / 5);
            // elevatorAccelerationTable.put(0.762, 7.274717623 / 5);
            elevatorAccelerationTable.put(1.016, 6.370643237 / 5);
            elevatorAccelerationTable.put(1.27, 5.666439564 / 6);
            elevatorAccelerationTable.put(1.524, 5.102204373 / 7);
            elevatorAccelerationTable.put(1.778, 4.640342002 / 8);
            elevatorAccelerationTable.put(1.8288, 4.557930098 / 8);
        }

        public static class GyroConstants {
            public static final int kGyroPort = 9; // ONLY used if Gyro is a Pigeon
            public static final double kGyroZero = 0; // Angle to zero the gyro at in degrees
            public static final double kGyroMountYawOffset = 0; // Angle to configure the offset of the gyro yaw to in degrees (mountYaw)
        }

        public static class ModuleConstants {
            public static class Common {
                public static class Drive {
                    public static final int CurrentLimit = 60;
                    public static final double gearRatio = 6.75;
                    public static final double VoltageCompensation = 12;
                    public static final double MaxModuleSpeed = Units.feetToMeters(15.1); // Maximum attainable module speed, from the SDS website
                    public static final double WheelDiameter = Units.inchesToMeters(4); // Standard SDS wheel
                    public static final double WheelCOF = 1.0; // Coefficient of friction
                    public static final double PositionConversionFactor = WheelDiameter * Math.PI / gearRatio; // Converts from rotations to meters, calculates to be 0.04729
                    public static final double VelocityConversionFactor = PositionConversionFactor / 60; // Converts from RPM to meters per second, calculates to be 0.0007881

                    // PID constants
                    public static final double kP = 0.035;
                    public static final double kI = 0.000;
                    public static final double kD = 0.0012;

                    // Feedforward constants
                    public static final double kV = 2.78;
                    public static final double kS = 0.0;
                    public static final double kA = 0.0;
                }
    
                public static class Turn {
                    public static final int CurrentLimit = 60;
                    public static final double VoltageCompensation = 12;
                    public static final double gearRatio = 12.8;
                    // TODO: check that radians for conversion factors don't break anything
                    public static final double RelativePositionConversionFactor =  (1 / gearRatio) * 2 * Math.PI; // Converts from rotations to radians, calculates out to be 0.4909
                    public static final double AbsolutePositionConversionFactor = 2*Math.PI;
                    public static final double VelocityConversionFactor = RelativePositionConversionFactor / 60; // Converts from RPM to radians/second

                    // PID constants
                    public static final double kP = 3;
                    public static final double kI = 0.0;
                    public static final double kD = 0.05;

                    // Feedforward constant
                    public static final double kS = 0.012009; // May be better just to leave this as zero
                }
            }
            public static enum ModuleConfig {
                FrontLeft(1,11,21,0.47314453125),
                FrontRight(2,12,22,-0.925048828125),
                RearRight(3,13,23,-0.439208984375),
                RearLeft(4,14,24,-0.41259765625);
    
                public final int DrivePort;
                public final int TurnPort;
                public final int EncoderPort;
                public final double EncoderOffsetRots;
    
                private ModuleConfig(int DrivePort, int TurnPort,int EncoderPort,double EncoderOffsetRots) {
                    this.DrivePort = DrivePort;
                    this.TurnPort = TurnPort;
                    this.EncoderPort = EncoderPort;
                    this.EncoderOffsetRots = EncoderOffsetRots;
                }
            }
        }

        public static class SimConstants {
            public static double kDriveKP = 0.005;
            public static double kDriveKD = 0;

            public static double kTurnKP = 8;
            public static double kTurnKD = 0;
        }
    }
}
