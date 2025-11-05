package frc.robot;

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

    public static class CANRangeConstants {
        public static final int grabberCANRangeId = 6;
        public static final double grabberCANRangeTriggerDistanceMeters = 0.0762;
    }
}
