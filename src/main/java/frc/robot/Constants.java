package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.SimpleControllerConstants;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.List;
import java.util.Map;

public class Constants {
    public enum RobotMode {
        /**
         * Running on a real robot
         */
        REAL,

        /** Running on a simulator */
        SIM,

        /** Replaying from a log file */
        REPLAY
    }

    public static class General {
        public static final RobotMode kSimMode = RobotMode.SIM;
        public static final RobotMode kRobotMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    //region Subsystems
    public static class Arm {
        public static final int kCanCoderID = 42;
        public static final double kCanCoderOffset = 1.010498 - Units.degreesToRotations(2);
        public static final SensorDirectionValue kCanCoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 40;
            kControllerConstants.real.main.inverted = true;
            kControllerConstants.real.currentLimit = 60;
            kControllerConstants.real.isBrakeMode = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createProfiledPID(120, 0, 0, 0, 2, 2, 0, 0, 0, 0.3, GravityTypeValue.Arm_Cosine);
            kControllerConstants.real.gearRatio = 86.4;
//            kControllerConstants.real.conversionFactor = 2 * Math.PI;
            kControllerConstants.real.homePosition = Units.degreesToRotations(-90);
            kControllerConstants.real.positionGoalTolerance = Units.degreesToRotations(4);

            /* Soft Limits */
            kControllerConstants.real.maxSoftLimit = Units.degreesToRotations(360);
            kControllerConstants.real.minSoftLimit = Units.degreesToRotations(-360);

            /* Hard Limit */
//            kControllerConstants.real.isLimitSwitch = true;
//            kControllerConstants.real.limitSwitchID = 1;
//            kControllerConstants.real.limitSwitchDirection = -1;
//            kControllerConstants.real.limitSwitchAutoStopReset = true;
//            kControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        private static final double DownAngle = -30;
        public static final Rotation2d[] LPositions = { Rotation2d.fromDegrees(Arm.Positions.Close.get()), Rotation2d.fromDegrees(Arm.Positions.L2.get()), Rotation2d.fromDegrees(Arm.Positions.L3.get()), Rotation2d.fromDegrees(Arm.Positions.L4.get()) };
        public static final Rotation2d[] LPositionsDown = { Rotation2d.fromDegrees(Arm.Positions.Close.get()), Rotation2d.fromDegrees(Arm.Positions.L2.get() + DownAngle), Rotation2d.fromDegrees(Arm.Positions.L3.get() + DownAngle), Rotation2d.fromDegrees(Arm.Positions.L4.get() + DownAngle) };
        public static final Rotation2d[] LPositionsInverse = { Rotation2d.fromDegrees(Arm.Positions.Close.get()), Rotation2d.fromDegrees(Arm.Positions.L2Inverse.get()), Rotation2d.fromDegrees(Arm.Positions.L3Inverse.get()), Rotation2d.fromDegrees(Arm.Positions.L4Inverse.get()) };
        public static final Rotation2d[] LPositionsDownInverse = { Rotation2d.fromDegrees(Arm.Positions.Close.get()), Rotation2d.fromDegrees(Arm.Positions.L2Inverse.get() - DownAngle), Rotation2d.fromDegrees(Arm.Positions.L3Inverse.get() - DownAngle), Rotation2d.fromDegrees(Arm.Positions.L4Inverse.get() - DownAngle) };
        public enum Positions {
            Close(-90),
            L2(-315),
            L3(-315),
            L4(-315),
            L2Inverse(-225),
            L3Inverse(-225),
            L4Inverse(-225),
            IntakeAlgae(-15),
            IntakeAlgaeLow(-15),
            IntakeAlgaeHigh(0),
            Net(110),
            NetInverse(70),
            Processor(0),
            IntakeCoral(-90),
            CoralReady(-270);

            final double angle;

            Positions(double angle) {
                this.angle = angle;
            }

            public double get() {
                return angle;
            }
        }
    }

    public static class Elevator {
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 30;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 60;
            kControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kControllerConstants.real.followers = new SimpleControllerConstants[1];
            kControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kControllerConstants.real.followers[0].id = 31;
            kControllerConstants.real.followers[0].inverted = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createProfiledPID(20, 0, 0, 0, 14, 100, 0, 0.7, 0.3, 0.3, GravityTypeValue.Elevator_Static);
            kControllerConstants.real.gearRatio = 6;
//            kControllerConstants.real.conversionFactor = Math.PI * 0.05; // Fix
            kControllerConstants.real.homePosition = 0;
            kControllerConstants.real.positionGoalTolerance = 0.05;

            /* Soft Limits */
            kControllerConstants.real.maxSoftLimit = 10.8;

            /* Hard Limit */
            kControllerConstants.real.isLimitSwitch = true;
//            kControllerConstants.real.isVirtualLimit = true;
//            kControllerConstants.real.virtualLimitStallThreshold = 30 / 12.0;
            kControllerConstants.real.limitSwitchID = 7;
            kControllerConstants.real.limitSwitchDirection = -1;
            kControllerConstants.real.limitSwitchAutoStopReset = true;
            kControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }

        public static final double[] LPositions = { Elevator.Positions.Close.get(), Elevator.Positions.L2.get(), Elevator.Positions.L3.get(), Elevator.Positions.L4.get() };
        public static final double[] LPositionsDown = { Elevator.Positions.Close.get(), Elevator.Positions.L2.get() - 1, Elevator.Positions.L3.get() - 1, Elevator.Positions.L4.get() - 1 };
        public enum Positions {
            Close(6.5),
            L2(2.25),
            L3(5.5),
            L4(10.7),
            AlgaeReefHigh(7.5),
            AlgaeReefLow(4.3),
            Net(10.7),
            AlgaeLow(0.3),
            Intake(6),
            CoralReady(1.6);

            final double height;

            Positions(double height) {
                this.height = height;
            }

            public double get() {
                return height;
            }
        }
    }

    public static class Outtake {
        public static final double kCurrentThreshold = 50;

        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 50;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 60;
            kControllerConstants.real.isBrakeMode = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Speeds {
            Intake(-0.8),
            Outtake(0.5),
            IntakeAlgae(-1),
            OuttakeAlgae(1);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class Intake {
        public static final int kBeamBreakerPort = 8;
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 20;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 80;
            kControllerConstants.real.isBrakeMode = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createTorqueCurrent(20, 0);

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Speeds {
            Intake(-70),
            Outtake(60),
            OuttakeL1(26);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class IntakeAngle {
        public static final int kCanCoderID = 23;
        public static final double kCanCoderOffset = 1.322266 - 0.268311;
        public static final SensorDirectionValue kCanCoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 21;
            kControllerConstants.real.main.inverted = true;
            kControllerConstants.real.currentLimit = 50;
            kControllerConstants.real.isBrakeMode = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 7, 35, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
            kControllerConstants.real.gearRatio = 65 + 1 / 3.0;
            kControllerConstants.real.conversionFactor = 2 * Math.PI;
            kControllerConstants.real.homePosition = Units.degreesToRadians(0);
            kControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(3);

            /* Soft Limits */
//            kControllerConstants.real.minSoftLimit = Units.degreesToRadians(-1000);
//            kControllerConstants.real.maxSoftLimit = Units.degreesToRadians(1000);

            /* Hard Limit */
            //        kIntakeAngleControllerConstants.real.isLimitSwitch = true;
            //        kIntakeAngleControllerConstants.real.limitSwitchID = 3;
            //        kIntakeAngleControllerConstants.real.limitSwitchDirection = -1;
            //        kIntakeAngleControllerConstants.real.limitSwitchAutoStopReset = true;
            //        kIntakeAngleControllerConstants.real.limitSwitchInverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Positions {
            Intake(-23 - 35 - 14 + 5 - 0.5),
            L1(34),
            Arm(65 - 0.5),
            Close(65 - 0.5),
            Algae(0);

            private final double degrees;

            Positions(double degrees) {
                this.degrees = degrees;
            }

            public double get() {
                return degrees;
            }
        }
    }

    public static class IntakeAligner {
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 22;
            kControllerConstants.real.main.inverted = true;
            kControllerConstants.real.currentLimit = 80;
            kControllerConstants.real.isBrakeMode = true;

            /* Control */
            kControllerConstants.real.controlConstants = ControlConstants.createTorqueCurrent(20, 0);

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(1);
        }

        public enum Speeds {
            Align(100);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }

    public static class Climber {
        public static final ControllerConstants kControllerConstants = new ControllerConstants();
        static {
            /* Base */
            kControllerConstants.real.main.id = 60;
            kControllerConstants.real.main.inverted = false;
            kControllerConstants.real.currentLimit = 80;
            kControllerConstants.real.isBrakeMode = true;

            /* Followers */
            kControllerConstants.real.followers = new SimpleControllerConstants[1];
            kControllerConstants.real.followers[0] = new SimpleControllerConstants();
            kControllerConstants.real.followers[0].id = 61;
            kControllerConstants.real.followers[0].inverted = true;

            /* Simulation */
            kControllerConstants.motorType = DCMotor.getKrakenX60(2);
        }
    }
    //endregion

    public static class Swerve {
        public static final double kDriverSpeedFactor = 1;
        public static final double kDriverRotationSpeedFactor = 1;
        public static final double kJoystickDeadband = 0.05;
        public static final boolean kDriverFieldRelative = true;

        public static final SwerveConstants kSwerveConstants = new SwerveConstants();
        static {
            /* Chassis */
            kSwerveConstants.chassis.trackWidth = 0.735;
            kSwerveConstants.chassis.wheelBase = 0.735;
            kSwerveConstants.chassis.bumperLength = 0.896;
            kSwerveConstants.chassis.bumperWidth = 0.896;
            kSwerveConstants.chassis.kinematics = new SwerveDriveKinematics(
                new Translation2d(kSwerveConstants.chassis.wheelBase / 2.0, kSwerveConstants.chassis.trackWidth / 2.0),
                new Translation2d(kSwerveConstants.chassis.wheelBase / 2.0, -kSwerveConstants.chassis.trackWidth / 2.0),
                new Translation2d(-kSwerveConstants.chassis.wheelBase / 2.0, kSwerveConstants.chassis.trackWidth / 2.0),
                new Translation2d(-kSwerveConstants.chassis.wheelBase / 2.0, -kSwerveConstants.chassis.trackWidth / 2.0)
            );

            /* Limits */
            kSwerveConstants.limits.maxSpeed = 4;
            kSwerveConstants.limits.maxAngularVelocity = 9.2;
            kSwerveConstants.limits.speedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationSpeedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.accelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationAccelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.maxSkidAcceleration = 65;

            /* Modules */
            double wheelRadius = 0.048;
            kSwerveConstants.modules.openLoop = false;
            kSwerveConstants.modules.driveMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.driveMotorConstants.real.currentLimit = 100;
            kSwerveConstants.modules.driveMotorConstants.real.gearRatio = 5.9;
            kSwerveConstants.modules.driveMotorConstants.real.conversionFactor = wheelRadius * 2 * Math.PI;
//            kSwerveConstants.modules.driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(90, 6);
            kSwerveConstants.modules.driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(7, 0);

            kSwerveConstants.modules.steerMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.steerMotorConstants.real.currentLimit = 60;
            kSwerveConstants.modules.steerMotorConstants.real.gearRatio = 18.75;
            kSwerveConstants.modules.steerMotorConstants.real.conversionFactor = 2 * Math.PI;
            kSwerveConstants.modules.steerMotorConstants.real.controlConstants = ControlConstants.createPID(10, 0, 0, 0);

            kSwerveConstants.modules.driveControllerType = Controller.ControllerType.TalonFX;
            kSwerveConstants.modules.steerControllerType = Controller.ControllerType.TalonFX;
            kSwerveConstants.modules.moduleConstants = new SwerveModuleConstants[4];

            for (int i = 0; i < 4; i++) {
                kSwerveConstants.modules.moduleConstants[i] = new SwerveModuleConstants(
                    i,
                    10 + i * 2,
                    11 + i * 2,
                    false,
                    false,
                    6 + i,
                    false,
                    0
                );
            }

            kSwerveConstants.modules.moduleConstants[0].CANCoderOffset = -0.291260;
            kSwerveConstants.modules.moduleConstants[1].CANCoderOffset = -0.268799;
            kSwerveConstants.modules.moduleConstants[2].CANCoderOffset = -0.267334;
            kSwerveConstants.modules.moduleConstants[3].CANCoderOffset = 0.279541;

            /* Gyro */
            kSwerveConstants.gyro.gyroID = 45;
            kSwerveConstants.gyro.gyroInverted = false;
            kSwerveConstants.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

            /* Simulation */
            kSwerveConstants.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
            kSwerveConstants.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);

            /* Special */
            kSwerveConstants.special.enableOdometryThread = true;
            kSwerveConstants.special.odometryThreadFrequency = 250;
            kSwerveConstants.special.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kSwerveConstants.special.robotStartPose = new Pose2d(3, 3, Rotation2d.kZero);
            kSwerveConstants.special.CANBus = "Swerve Bus";

            try {
                kSwerveConstants.special.robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
        static {
            kSwerveControllerConstants.swerveConstants = kSwerveConstants;
            kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(5, 0, 0, 0);
            kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(6, 0.5, 0, Units.degreesToRadians(5));
            kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
        }

        public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
            new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
            new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
        );
    }

    public static class Vision {
        public static final double kMaxDistanceFilter = 2;
        public static final double kMinDistanceFilter = 0.6;
        public static final double kMaxSpeedFilter = 3;
        public static final double kMaxAngularSpeedFilter = 7;
        public static final double kMaxAmbiguityFilter = 0.2;
        public static final double kOdometryDriftPerMeter = 0.02;
        public static final double kCrashAcceleration = 15;
        public static final double kOdometryDriftPerCrash = 0.5;

        public static final VisionConstants kVisionConstants = new VisionConstants();
        static {
            kVisionConstants.cameras = Map.of(
                "Front", Pair.of(new Transform3d(0.28286, 0.12995, 0.152, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-7.5))), VisionConstants.CameraType.PhotonVision),
                "Back", Pair.of(new Transform3d(-0.28286 - 0.05, 0.12995, 0.152, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(187.5))), VisionConstants.CameraType.PhotonVision)
            );

            kVisionConstants.fieldLayoutGetter = Constants.Field::getFieldLayoutWithIgnored;
            kVisionConstants.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kVisionConstants.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
        }

        public static Matrix<N3, N1> getVisionSTD(VisionOutput output) {
//            double distStd = Math.pow(0.5/*0.8*/ * output.closestTargetDist, 2) + 0.3;
            double distStd = 0.185185 * Math.pow(output.closestTargetDist, 2);

            ChassisSpeeds speed = frc.lib.NinjasLib.swerve.Swerve.getInstance().getChassisSpeeds(false);
            double xySpeedStd = 4/*6*/ * output.latency * Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
            double angleSpeedStd = 4/*6*/ * output.latency * speed.omegaRadiansPerSecond;

            double xyStd = distStd + xySpeedStd;
            double angleStd = distStd + angleSpeedStd;

            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ latency", output.latency);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ dist", output.closestTargetDist);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ distStd", distStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ vel", Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond));
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ velStd", xySpeedStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ angleVel", speed.omegaRadiansPerSecond);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ angleVelStd", angleSpeedStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ xyStd", xyStd);
            Logger.recordOutput("Vision/Stds/" + output.cameraName + "/ angleStd", angleStd);
            return VecBuilder.fill(xyStd, xyStd, angleStd);
        }
    }

    public static class Field {
        public static AprilTagFieldLayout kBlueFieldLayout;
        public static AprilTagFieldLayout kRedFieldLayout;
        private static List<Pose2d> reefAprilTags;

        static {
            try {
                kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
                kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

                kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
                kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);

//                // Switch tag 14 to be 18 for testing. REMOVE BEFORE COMP
//                List<AprilTag> blueTags = kBlueFieldLayout.getTags();
//                blueTags.set(14 - 1, new AprilTag(14, blueTags.get(18 - 1).pose));
//                blueTags.set(9 - 1, new AprilTag(9, blueTags.get(18 - 1).pose));
//                kBlueFieldLayout = new AprilTagFieldLayout(blueTags, kBlueFieldLayout.getFieldLength(), kBlueFieldLayout.getFieldWidth());
//                kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
//
//                List<AprilTag> redTags = kRedFieldLayout.getTags();
//                redTags.set(14 - 1, new AprilTag(14, redTags.get(18 - 1).pose));
//                redTags.set(9 - 1, new AprilTag(9, redTags.get(18 - 1).pose));
//                kRedFieldLayout = new AprilTagFieldLayout(redTags, kRedFieldLayout.getFieldLength(), kRedFieldLayout.getFieldWidth());
//                kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            } catch (IOException e) {
                throw new RuntimeException("Unable to load field layout");
            }

            AprilTagFieldLayout layout = Constants.Field.getFieldLayout();
            reefAprilTags = List.of(
                layout.getTagPose(6).get().toPose2d(),
                layout.getTagPose(7).get().toPose2d(),
                layout.getTagPose(8).get().toPose2d(),
                layout.getTagPose(9).get().toPose2d(),
                layout.getTagPose(10).get().toPose2d(),
                layout.getTagPose(11).get().toPose2d(),
                layout.getTagPose(17).get().toPose2d(),
                layout.getTagPose(18).get().toPose2d(),
                layout.getTagPose(19).get().toPose2d(),
                layout.getTagPose(20).get().toPose2d(),
                layout.getTagPose(21).get().toPose2d(),
                layout.getTagPose(22).get().toPose2d()
            );
        }

        public static AprilTagFieldLayout getFieldLayoutWithIgnored(List<Integer> ignoredTags) {
            AprilTagFieldLayout layout;

            layout = RobotState.getAlliance() == DriverStation.Alliance.Blue
                ? kBlueFieldLayout
                : kRedFieldLayout;

            if (!ignoredTags.isEmpty()) {
                List<AprilTag> tags = layout.getTags();
                tags.removeIf(tag -> ignoredTags.contains(tag.ID));
                layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
            }

            return layout;
        }

        public static AprilTagFieldLayout getFieldLayoutWithAllowed(List<Integer> allowedTags) {
            AprilTagFieldLayout layout = getFieldLayout();
            if (!allowedTags.isEmpty()) {
                List<AprilTag> tags = layout.getTags();
                tags.removeIf(tag -> !allowedTags.contains(tag.ID));
                layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
            }

            return layout;
        }

        public static AprilTagFieldLayout getFieldLayout() {
            return getFieldLayoutWithIgnored(List.of());
        }

        public static Pose3d getTagPose(int id) {
            return getFieldLayout().getTagPose(id).get();
        }

        public static AprilTag nearestReef() {
            AprilTagFieldLayout layout = Constants.Field.getFieldLayout();
            List<Integer> reefAprilTagsIds = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
            Pose2d nearest = RobotState.getInstance().getRobotPose().nearest(reefAprilTags);

            for (int id : reefAprilTagsIds) {
                if (Math.abs(getTagPose(id).toPose2d().getX() - nearest.getX()) < 0.01 && Math.abs(getTagPose(id).toPose2d().getY() - nearest.getY()) < 0.01)
                    return new AprilTag(id, getTagPose(id));
            }

            return layout.getTags().get(0);
        }
    }

    public static class AutoDrive {
        public static double kReefRodOffset = 0.3302 / 2;

        public static double kDistFromReef = 0.56 - 0.005 - 0.01;
        public static double kDistFromReefInverse = 0.56 - 0.005 - 0.01;
        public static double kDistFromReefL4 = 0.6 - 0.005 - 0.01;

        public static double kRightSideOffset = 0 - 0.1651 - 0.01;
        public static double kLeftSideOffset = 0.34 - 0.1651;
        public static double kRightSideInverseOffset = 0 - 0.1651 + 0.40 - 0.005;
        public static double kLeftSideInverseOffset = 0 + 0.1651 - 0.40 + 0.33 - 0.36 + 0.03 + 0.005;

        public static double kRightSideL2ExtraOffset = 0;
        public static double kLeftSideL2ExtraOffset = 0;
        public static double kRightSideInverseL2ExtraOffset = -0.01;
        public static double kLeftSideInverseL2ExtraOffset = 0.01;

        public static double kDistThreshold = 0.01;
        public static Rotation2d kAngleThreshold = Rotation2d.fromDegrees(1.5);

        public static double kDistBackFirstTarget = 0.2;
        public static double kFirstDistThreshold = 0.08;

        static {
            boolean isSadna = false;

            if(isSadna) {
                kDistFromReef = 0.545;
                kDistFromReefL4 = 0.585;
                kRightSideOffset = -0.05 - 16.51;
                kLeftSideOffset = 0.35 - 16.51;
                kRightSideInverseOffset = -0.05 - 16.51;
                kLeftSideInverseOffset = 0.35 - 16.51;
            }
        }
    }
}
