package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SuperstructureConstants.WristevatorState;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import org.apache.commons.lang3.NotImplementedException;
import java.util.Set;

import static frc.robot.utils.Localization.flipPose;
import static frc.robot.utils.Localization.mirrorPose;

public final class Constants 
{
     
     public static class VisionConstants 
     {
        public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        public static final Set<Integer> filteredTargetIDs = Set.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        public static class Photonvision 
        {

            public static final String driverCamName = "DRIVER_CAM";

            public static final Vector<N3> kDefaultSingleTagStdDevs = VecBuilder.fill(1, 1, 1);
            public static final Vector<N3> kDefaultMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);

            public static enum PhotonConfig 
            {
                
                FRONT_LEFT_CAM(
                    "FRONT_LEFT_CAM", 
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    13.75, 11.683, 11.25,
                    0, 0, -20
                ),
                FRONT_RIGHT_CAM(
                    "FRONT_RIGHT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    13.75, -11.683, 11.25,
                    0, 0, 20
                ),
                BACK_RIGHT_CAM(
                    "BACK_RIGHT_CAM",
                    kDefaultSingleTagStdDevs,
                    kDefaultMultiTagStdDevs,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    PoseStrategy.PNP_DISTANCE_TRIG_SOLVE,
                    -12.785, -12.565, 6.25+4.411,
                    0, 0, -140
                );

                public final String name;
                public final Transform3d offset;
                public final Matrix<N3, N1> defaultSingleTagStdDevs;
                public final Matrix<N3, N1> defaultMultiTagStdDevs;
                public final PoseStrategy multiTagPoseStrategy;
                public final PoseStrategy singleTagPoseStrategy;
                private PhotonConfig(
                    String name, 
                    Matrix<N3, N1> defaultSingleTagStdDevs,
                    Matrix<N3, N1> defaultMultiTagStdDevs,
                    PoseStrategy multiTagPoseStrategy,
                    PoseStrategy singleTagPoseStrategy,
                    double xInch, double yInch, double zInch, 
                    double rollDeg, double pitchDeg, double yawDeg
                ) 
                {
                    this.name = name;
                    this.offset = new Transform3d(
                        Units.inchesToMeters(xInch),
                        Units.inchesToMeters(yInch),
                        Units.inchesToMeters(zInch),
                        new Rotation3d(
                            Units.degreesToRadians(rollDeg),
                            Units.degreesToRadians(pitchDeg),
                            Units.degreesToRadians(yawDeg)
                        )
                    );
                    this.multiTagPoseStrategy = multiTagPoseStrategy;
                    this.singleTagPoseStrategy = singleTagPoseStrategy;
                    this.defaultMultiTagStdDevs = defaultMultiTagStdDevs;
                    this.defaultSingleTagStdDevs = defaultSingleTagStdDevs;
                }
            }
        }
    }
  
    public static class GameConstants 
    {

        public static Alliance teamColor = Alliance.Blue;
        public static AutonSides autonSide = AutonSides.Left;
        public static boolean rearRightCameraEnabledAuton = false;
        
        public enum TeamColors 
        {
            kTeamColorBlue("BLUE"),
            kTeamColorRed("RED");

            public final String color;

            private TeamColors(String color) 
            {
                this.color = color;
            }
        }
        
        public enum AutonSides 
        {
            Left(false),
            Right(true);

            public final boolean isRightSide;

            private AutonSides (boolean isRightSide) 
            {
                this.isRightSide = isRightSide;
            }
        }
    }

    public static class OIConstants 
    {
        public static final boolean kUseDroperatorController = true;
        public static final boolean kUseOperiverColorController = true;

        public static final boolean kUseAlternateDriverController = true;
        public static final boolean kUseAlternateOperatorController = false;

        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kDroperatorControllerPort = 2;
        public static final double kControllerDeadband = 0.15;
        public static final double kControllerTriggerThreshold = 0.7;
    }

    public static class SystemConstants 
    {
        public static final int currentMode = 0;
        public static final boolean operatorSysID = false;
        public static final boolean driverSysID = false;
        public static final boolean logOdometry = false;
        public static final boolean logCTRE = false;
    }
    
    public static class DriveConstants 
    {

        public static class DriverControlConstants 
        {
            public static final double singleClutchTranslationFactor = 0.5;
            public static final double singleClutchRotationFactor = 0.5;
            public static final double doubleClutchTranslationFactor = 0.3;
            public static final double doubleClutchRotationFactor = 0.35;
            public static final double FPVClutchTranslationFactor = 0.1;
            public static final double FPVClutchRotationFactor = 0.1;
            public static final double ElevatorClutchTransFactor = 0.5;
            public static final double ElevatorClutchRotFactor = 0.5;
            public static final double maxTranslationSpeedMPS = 3.0;
            public static final double maxRotationSpeedRadPerSec = 3.0;

            public static final InterpolatingDoubleTreeMap elevatorAccelerationTable = new InterpolatingDoubleTreeMap();
            static 
            {
                elevatorAccelerationTable.put(0.0,100000.0);
                elevatorAccelerationTable.put(1.0,100000.0);
                elevatorAccelerationTable.put(1.016, 6.370643237 / 5);
                elevatorAccelerationTable.put(1.27, 5.666439564 / 6);
                elevatorAccelerationTable.put(1.524, 5.102204373 / 7);
                elevatorAccelerationTable.put(1.778, 4.640342002 / 8);
                elevatorAccelerationTable.put(1.8288, 4.557930098 / 8);
            }
        }

        public static class DirectDriveConstants 
        {
            public static final Constraints translationConstraints = new Constraints(2, 2);
            public static final Constraints headingConstraints = new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(360));
        }

        public static class PathPlannerConstants 
        {
            public static final PathConstraints pathConstraints = new PathConstraints(2, 2, Units.degreesToRadians(360), Units.degreesToRadians(360));
            public static final PathConstraints netConstraints = new PathConstraints(2, 2.6788, Units.degreesToRadians(360), Units.degreesToRadians(360));
            
            public static final Transform2d robotOffset = new Transform2d(0.508, 0, Rotation2d.kZero);
            public static final Transform2d robotLeftL1Offset = new Transform2d(0.508, Units.inchesToMeters(-12), Rotation2d.kZero);
            public static final Transform2d robotCenterL1Offset = new Transform2d(0.508, Units.inchesToMeters(0), Rotation2d.kZero);
            public static final Transform2d robotRightL1Offset = new Transform2d(0.508, Units.inchesToMeters(2), Rotation2d.kZero);
            
            public static final double pathGenerationToleranceMeters = 0.011;
            public static final double LEDpathToleranceMeters = 0.03;

            public static class Control 
            {
                public static final PIDConstants transPID = new PIDConstants(5, 0, 0);
                public static final PIDConstants rotPID = new PIDConstants(5, 0, 0);
            }
        }

        public static final double kOdometryUpdateFrequency = 250.0;
    }

    public static class SuperstructureConstants 
    {

        public static Rotation2d algaeTravelAngle = Rotation2d.fromDegrees(70);
        public static Rotation2d coralTravelAngle = Rotation2d.fromDegrees(90);

        public enum GrabberPossession 
        {
            EMPTY(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG),
            CORAL(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG),
            ALGAE(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG 
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG 
                    : ElevatorSimConstants.Control.kG),
            TRANSFERRING(
                SystemConstants.currentMode == 0
                    ? WristConstants.Control.kG
                    : WristSimConstants.Control.kG,
                SystemConstants.currentMode == 0
                    ? ElevatorConstants.Control.kG
                    : ElevatorSimConstants.Control.kG);

            public final double wrist_kG;
            public final double elevator_kG;

            private GrabberPossession(double wrist_kG, double elevator_kG) 
            {
                this.wrist_kG = wrist_kG;
                this.elevator_kG = elevator_kG;
            }
        }

        public enum GrabberState 
        {

            IDLE(0, 0),
            
            ALGAE_INTAKE(-12, -12),
            ALGAE_HOLD(-2, -2),
            CORAL_INTAKE(12, 12),
            REVERSE_CORAL_INTAKE(-12, -12),
            GRABBER_CORAL_INTAKE(-12, -12),

            ALGAE_OUTTAKE(12, 12),
            CORAL_OUTTAKE(12, 12),
            L1_OUTTAKE(8, 5),
            DEFAULT_OUTTAKE(12, 12);

            public final double leftVoltage;
            public final double rightVoltage;

            private GrabberState(double leftVoltage, double rightVoltage) 
            {
                this.leftVoltage = leftVoltage;
                this.rightVoltage = rightVoltage;
            }
        }

        public enum IndexState
        {
            TRANSFER(6),
            IDLE(0),
            BACKWARDS(-1);
            
            public final double volts;

            private IndexState(double volts) 
            {
                this.volts = volts;
            }
        }

        public enum WristevatorState 
        {
            
            TRAVEL(0.1349839121 + 0.00635, 90),
            ALGAE_TRAVEL(0.134983912 + 0.00635, 80),
            PRE_AUTOMATIC_NET(0.1349839121 + 0.00635, 45),

            CORAL_TRANSFER(0.1349839121 + 0.00635, -15.57789 + 2),
            GRABBER_CORAL_INTAKE(0.784, 36.956),
            GRABBER_CORAL_INTAKE_TRAVEL(0.784, 90),
            HIGH_TRAVEL(0.3, -90),

            L1(0.394, 23.9365),
            L1_STACK(0.3048 + 3 * 0.0254, 15),
            L2(0.910, -35),
            L3(1.348 + 2 * 0.00889, -35),
            L4(2.11455, -38),

            GROUND_INTAKE(0, 0),
            LOLLIPOP_INTAKE(0.1349839121 + 0.00635, 26.7),
            LOW_INTAKE(0.98407,-27.02),
            HIGH_INTAKE(1.44998, -27.02),

            PROCESSOR(0.2536, 0),
            NET(2.109649 + 3 * 0.00635, 67.5);

            public final double elevatorHeightMeters;
            public final Rotation2d wristAngle;
            
            private WristevatorState(double elevatorHeightMeters, double wristAngleDegrees) 
            {
                this.elevatorHeightMeters = elevatorHeightMeters;
                this.wristAngle = Rotation2d.fromDegrees(wristAngleDegrees);
            }
        }
    }

    public static class FieldConstants 
    {
        public static final double fieldWidthMeters = Units.inchesToMeters(316.63);
        public static final double fieldLengthMeters = 17.54;

        private static final double branchOffset = Units.inchesToMeters(6.469);
        private static final Transform2d leftBranchTransform = new Transform2d(0.0, -branchOffset, Rotation2d.kZero);
        private static final Transform2d rightBranchTransform = new Transform2d(0.0, branchOffset, Rotation2d.kZero);

        private static final Transform2d leftL1Transform = new Transform2d(0.0, Units.inchesToMeters(-19), Rotation2d.kZero);
        private static final Transform2d middleL1Transform = new Transform2d(0.0, Units.inchesToMeters(-7), Rotation2d.kZero);
        private static final Transform2d rightL1Transform = new Transform2d(0.0, Units.inchesToMeters(5), Rotation2d.kZero);
        private static final Transform2d leftL1StackTransform = new Transform2d(0.0, Units.inchesToMeters(-13), Rotation2d.kZero);
        private static final Transform2d rightL1StackTransform = new Transform2d(0.0, Units.inchesToMeters(-1), Rotation2d.kZero); 

        public enum ReefFace 
        {
            // Blue Reef
            BLU_REEF_AB(18, 3.657600, 4.025900, 180.0, null, null, true),
            BLU_REEF_CD(17, 4.073906, 3.306318, 240.0, null, null, false),
            BLU_REEF_EF(22, 4.904740, 3.306318, 300.0, null, null, true),
            BLU_REEF_GH(21, 5.321046, 4.025900, 0.0, null, null, false),
            BLU_REEF_IJ(20, 4.904740, 4.745482, 60.0, null, null, true),
            BLU_REEF_KL(19, 4.073906, 4.745482, 120.0, null, null, false),

            // Red Reef
            RED_REEF_AB(7, 13.890498, 4.025900, 0.0, null, null, true),
            RED_REEF_CD(8, 13.474446, 4.745482, 60., null, null, false),
            RED_REEF_EF(9, 12.643358, 4.745482, 120.0, null, null, true),
            RED_REEF_GH(10, 12.227306, 4.025900, 180.0, null, null, false),
            RED_REEF_IJ(11, 12.643358, 3.306318, 240.0, null, null, true),
            RED_REEF_KL(6, 13.474446, 3.306318, 300.0, null, null, false);


            public final Double leftBranchFudgeTransform;
            public final Double rightBranchFudgeTransform;

            public final Pose2d leftBranch;
            public final Pose2d rightBranch;
            public final Pose2d leftL1;
            public final Pose2d middleL1;
            public final Pose2d rightL1;
            public final Pose2d leftStackL1;
            public final Pose2d rightStackL1;
            private int L1Index;

            public final Pose2d AprilTag;
            public final int aprilTagID;
            public final boolean algaeHigh;

            private ReefFace(int aprilTagID, double aprilTagX, double aprilTagY, double aprilTagTheta, Double leftBranchFudgeTransform, Double rightBranchFudgeTransform, boolean algaeHigh) 
            {
                this.aprilTagID = aprilTagID;
                this.AprilTag = new Pose2d(aprilTagX, aprilTagY, Rotation2d.fromDegrees(aprilTagTheta));
                this.leftBranchFudgeTransform = leftBranchFudgeTransform;
                this.rightBranchFudgeTransform = rightBranchFudgeTransform;

                this.leftL1 = AprilTag.transformBy(leftL1Transform);
                this.middleL1 = AprilTag.transformBy(middleL1Transform);
                this.rightL1 = AprilTag.transformBy(rightL1Transform);
                this.leftStackL1 = AprilTag.transformBy(leftL1StackTransform);
                this.rightStackL1 = AprilTag.transformBy(rightL1StackTransform);
                this.L1Index = 0;

                this.algaeHigh = algaeHigh;

                if (this.leftBranchFudgeTransform == null) 
                {
                    this.leftBranch = AprilTag.transformBy(leftBranchTransform);
                } 
                else 
                {
                    this.leftBranch = AprilTag.transformBy(new Transform2d(0.0, -this.leftBranchFudgeTransform, Rotation2d.kZero));
                }
                
                if (this.rightBranchFudgeTransform == null) 
                {
                    this.rightBranch = AprilTag.transformBy(rightBranchTransform);
                } 
                else 
                {
                    this.rightBranch = AprilTag.transformBy(new Transform2d(0.0, this.rightBranchFudgeTransform, Rotation2d.kZero));
                }
            }

            public Pose2d getNextL1Position() 
            {
                Pose2d L1Position;

                if (L1Index % 3 == 0) 
                {
                    L1Position = rightL1;
                } 
                else if (L1Index % 3 == 1) 
                {
                    L1Position = middleL1;
                } 
                else if (L1Index % 3 == 2) 
                {
                    L1Position = leftL1;
                } 
                else if (L1Index % 3 == 3) 
                {
                    L1Position = rightStackL1;
                } 
                else 
                {
                    L1Position = leftStackL1;
                }

                return L1Position;
            }

            public WristevatorState getNextL1WristevatorState() 
            {
                WristevatorState state;

                if (L1Index % 3 <= 2) 
                {
                    state = WristevatorState.L1;
                } else 
                {
                    state = WristevatorState.L1_STACK;
                }

                return state;
            }

            public void increaseL1Index() 
            {
                L1Index++;
            }
        }

        public enum PointOfInterest 
        {
            BLU_REEF(4.487, 4.010),
            RED_REEF(13.062, 4.010);

            public final Translation2d position;
            private PointOfInterest(double xMeters, double yMeters) 
            {
                this.position = new Translation2d(xMeters, yMeters);
            }
        }

        private static Pose2d defaultOuterStation = new Pose2d(1.585, 7.550, Rotation2d.fromDegrees(-49.992));
        private static Pose2d defaultInnerStation = new Pose2d(0.596, 6.807, Rotation2d.fromDegrees(-55));

        public enum PoseOfInterest 
        {
            BLU_PROCESSOR(5.973318, -0.00381, 90),
            RED_PROCESSOR(11.56081,	8.05561,	270),

            BLUE_RIGHT_OUTER_STATION(mirrorPose(defaultOuterStation)),
            BLUE_RIGHT_INNER_STATION(mirrorPose(defaultInnerStation)),
            BLUE_LEFT_OUTER_STATION(defaultOuterStation),
            BLUE_LEFT_INNER_STATION(defaultInnerStation),
            RED_RIGHT_OUTER_STATION(mirrorPose(flipPose(defaultOuterStation))),
            RED_RIGHT_INNER_STATION(mirrorPose(flipPose(defaultInnerStation))),
            RED_LEFT_OUTER_STATION(flipPose(defaultOuterStation)),
            RED_LEFT_INNER_STATION(flipPose(defaultInnerStation));

            public final Pose2d pose;

            private PoseOfInterest(double xMeters, double yMeters, double omegaDeg) 
            {
                this.pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(omegaDeg));
            }

            private PoseOfInterest(Pose2d pose) 
            {
                this.pose = pose;
            }
        }
    }

    public static class ElevatorConstants 
    {
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

        public static class Electrical 
        {
            public static final double kVoltageCompensation = 10.5; 
            public static final double kCurrentLimit = 60;
        }


        public static class Control 
        {
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

    public static class ElevatorSimConstants 
    {
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

    public static final class GrabberConstants 
    {
        public static final int kLeftMotorPort = 41;
        public static final int kRightMotorPort = 42;
        
        public static final double kCurrentLimit = 20; 
        public static final double kGearRatio = 20;
        public static final double kPositionConversionFactor = Math.PI * 2 * (1/kGearRatio);

        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;
    }

    public static class WristConstants 
    {
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

        public static final class Control 
        {
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

    public static class WristSimConstants 
    {
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

    public static class IndexConstants 
    {
        public static final int kLeadMotorPort = 51;

        public static final double kCurrentLimit = 20.0;

        public static final boolean kLeadMotorInverted = false;
        public static final boolean kFollowMotorInverted = true;
    }

    public static class BeamBreakConstants
    {
        public static final int transferBeamBreakPort = 0;
    }

    public static class CANRangeConstants 
    {
        public static final int grabberCANRangeId = 6;
        public static final double grabberCANRangeTriggerDistanceMeters = 0.0762;
    }

    public static class LEDConstants 
    {
        public static final double kHPSignalTime = 3.0;

        public static class LEDDIOConstants 
        {
            public static final int kDIOPort1 = 7;
            public static final int kDIOPort2 = 8;
            public static final int kDIOPort3 = 9;
        }

        public static class LEDOnRIOConstants 
        {
            public static final int kPWMPort = 0;
            public static final int kLength = 72;

            public static final double kFlashSeconds = 0.1;
            public static final int kEmptyStateBrightness = 100;
            public static final int kFlashingStateBrightness = 100;
        }

        public static enum LEDStates 
        {
            IDLE(false, false, false),
            CORAL_INDEXED(true, false, false),
            HUMAN_PLAYER_SIGNAL(false, true, false),
            ALGAE(true, true, false),
            AUTO_ALIGNED(false, false, true),
            AUTO_ALIGNING(false, false, false),
            OFF(true, false, true),
            ELEVATOR_ZEROED(false, true, true),
            
            RED_HP_SIGNAL(true, true, true),
            ORANGE_HP_SIGNAL(false,false,false),
            YELLOW_HP_SIGNAL(false,false,false),
            GREEN_HP_SIGNAL(false,false,false),
            BLUE_HP_SIGNAL(false,false,false),
            PURPLE_HP_SIGNAL(false,false,false),
            WHITE_HP_SIGNAL(false,false,false);

            public final boolean onesPlace;
            public final boolean twosPlace;
            public final boolean foursPlace;
            private LEDStates(boolean onesPlace, boolean twosPlace, boolean foursPlace) 
            {
                this.onesPlace = onesPlace;
                this.twosPlace = twosPlace;
                this.foursPlace = foursPlace;
            } 
        }
    }

    private Constants() 
    {
        throw new NotImplementedException();
    }
}
