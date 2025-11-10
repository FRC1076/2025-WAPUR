package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
        public static final int kOperatorControllerPort = 0;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class SystemConstants {
        public static final RobotMode currentMode = RobotMode.REAL;
        
        public static final boolean signalLoggerEnabled = false;
        public static final boolean increaseThreadPriority = true;

        public static enum RobotMode {
            REAL,
            SIM,
            REPLAY;
        }
    }

    public static class ElevatorConstants  {
        public static final int kMotorPort0 = 31;
        public static final int kMotorPort1 = 32;
        
        public static final double elevatorPositionToleranceMeters = Units.inchesToMeters(0.5);
        public static final double kMinElevatorHeightMeters = Units.inchesToMeters(0);
        public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(83.25);
        
        public static final double defaultMaxOperatorControlVolts = 1.5;
        public static final double fasterMaxOperatorControlVolts = 4;

        public static final boolean leadMotorInverted = false;
        public static final boolean followMotorInverted = false;

        public static final double kGearRatio = 10.909;
        public static final double kElevatorStages = 3;
        public static final double kVelocityConversionFactor = (24.0/22.0) * kElevatorStages * (1/kGearRatio) * 24 * 0.00635 / 60.0;
        public static final double kPositionConversionFactor = (24.0/22.0) * kElevatorStages * (1/kGearRatio) * 24 * 0.00635;

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

            public static final Constraints kProfileConstraints = new Constraints(3, 5.25);
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
        public static final int kLeftMotorPort = 41;
        public static final int kRightMotorPort = 42;
        
        public static final double kCurrentLimit = 20; 
        public static final double kGearRatio = 20;
        public static final double kPositionConversionFactor = Math.PI * 2 * (1/kGearRatio);

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;
    }

    public static class ShooterConstants {
        public static final int kMotorPort = 0;
        public static final int kServoPort = 0;

        public static class Control {
            public static final double kP = 0;
            public static final double kI = 0;
            public static final double kD = 0;

            public static final double kS = 0;
            public static final double kV = 0;
            public static final double kA = 0;
        }
    }

    public static class WristConstants {
        public static final int kLeadMotorPort = 61;

        public static final double wristAngleToleranceRadians = Units.degreesToRadians(2);
        public static final double kMinWristAngleRadians = Units.degreesToRadians(-90);
        public static final double kMaxWristAngleRadians = Units.degreesToRadians(90);

        public static final double maxOperatorControlVolts = 1;
        public static final double kSmartCurrentLimit = 20.0;

        public static final boolean kLeadMotorInverted = true;

        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2 * Math.PI;
        public static final double kVelocityConversionFactor = 2 * Math.PI / 60;
        public static final double kZeroOffsetRadians = -1.7236260000051038;

        public static final class Control {
            public static final double kP = 18;
            public static final double kI = 0.0;
            public static final double kD = 0;

            public static final double kS = 0.16629;
            public static final double kG = 0.13459;
            public static final double kV = 1.8105;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);
        }
    }

    public static class YoinkerConstants {
        public static final int kLeadMotorPort = 61;

        public static final double wristAngleToleranceRadians = Units.degreesToRadians(2);
        public static final double kMinYoinkerAngleRadians = Units.degreesToRadians(-90);
        public static final double kMaxYoinkerAngleRadians = Units.degreesToRadians(90);

        public static final double maxOperatorControlVolts = 1;
        public static final double kSmartCurrentLimit = 20.0;

        public static final boolean kLeadMotorInverted = true;

        public static final int kCountsPerRevolution = 8192;
        public static final double kPositionConversionFactor = 2 * Math.PI;
        public static final double kVelocityConversionFactor = 2 * Math.PI / 60;
        public static final double kZeroOffsetRadians = -1.7236260000051038;

        public static final class Control {
            public static final double kP = 18;
            public static final double kI = 0.0;
            public static final double kD = 0;

            public static final double kS = 0.16629;
            public static final double kG = 0.13459;
            public static final double kV = 1.8105;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(4 * Math.PI, 3.5 * Math.PI);
        }
    }

    public static class WristSimConstants {
        public static final double kWristGearingReductions = 125.0;
        public static final double kWristLength = Units.feetToMeters(1);
        public static final double kWristMass = 2.0;
        public static final double kMinAngleRads = -0.75 * Math.PI;
        public static final double kMaxAngleRads = 0.75 * Math.PI;

        public static class Control 
        {
            public static final double kP = 18.0;
            public static final double kI = 0.0;
            public static final double kD = 0.0;

            public static final double kS = 0.0;
            public static final double kG = 0.0553;
            public static final double kV = 0.0;
            public static final double kA = 0.0;

            public static final Constraints kProfileConstraints = new Constraints(2, 2);
        }


    }

    public static class IndexConstants {
        public static final int kLeadMotorPort = 51;

        public static final double kCurrentLimit = 20.0;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = true;
    }

    public static class BeamBreakConstants {
        public static final int transferBeamBreakPort = 0;
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

        public static class GyroConstants {
            public static final int kGyroPort = 9; // ONLY used if Gyro is a Pigeon
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
                FrontLeft(1,11,21,0.16259765625),
                FrontRight(2,12,22,-0.3017578125),
                RearRight(3,13,23,0.144287109375),
                RearLeft(4,14,24,0.236328125);
    
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
    }
}
