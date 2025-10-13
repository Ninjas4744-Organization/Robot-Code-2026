package frc.robot;

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
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
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
        /** Running on a real robot */
        REAL,

        /** Running on a simulator */
        SIM,

        /** Replaying from a log file */
        REPLAY
    }

    /* General */
    public static class General {
        public static final RobotMode kSimMode = RobotMode.SIM;
        public static final RobotMode kRobotMode = Robot.isReal() ? RobotMode.REAL : kSimMode;
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }


    /* Example Subsystems */
    public static final ControllerConstants kExampleSubsystemControllerConstants = new ControllerConstants();
    static {
        /* Base */
        kExampleSubsystemControllerConstants.real.main.id = 20;
        kExampleSubsystemControllerConstants.real.main.inverted = false;
        kExampleSubsystemControllerConstants.real.currentLimit = 60;
        kExampleSubsystemControllerConstants.real.isBrakeMode = true;

        /* Followers */
        kExampleSubsystemControllerConstants.real.followers = new RealControllerConstants.SimpleControllerConstants[1];
        kExampleSubsystemControllerConstants.real.followers[0] = new RealControllerConstants.SimpleControllerConstants();
        kExampleSubsystemControllerConstants.real.followers[0].id = 21;
        kExampleSubsystemControllerConstants.real.followers[0].inverted = true;

        /* Control */
        kExampleSubsystemControllerConstants.real.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
        kExampleSubsystemControllerConstants.real.gearRatio = 50;
        kExampleSubsystemControllerConstants.real.conversionFactor = 2 * Math.PI;
        kExampleSubsystemControllerConstants.real.homePosition = Units.degreesToRadians(-60);
        kExampleSubsystemControllerConstants.real.positionGoalTolerance = Units.degreesToRadians(1.5);

        /* Soft Limits */
        kExampleSubsystemControllerConstants.real.maxSoftLimit = Units.degreesToRadians(240);
        kExampleSubsystemControllerConstants.real.minSoftLimit = Units.degreesToRadians(40);

        /* Hard Limit */
        kExampleSubsystemControllerConstants.real.isLimitSwitch = true;
        kExampleSubsystemControllerConstants.real.limitSwitchID = 2;
        kExampleSubsystemControllerConstants.real.limitSwitchDirection = -1;
        kExampleSubsystemControllerConstants.real.limitSwitchAutoStopReset = true;
        kExampleSubsystemControllerConstants.real.limitSwitchInverted = true;
        kExampleSubsystemControllerConstants.real.isVirtualLimit = false;
        kExampleSubsystemControllerConstants.real.virtualLimitStallThreshold = 50 / 12.0;

        /* Simulation */
        kExampleSubsystemControllerConstants.motorType = DCMotor.getKrakenX60(2);
    }

    // Example System enum for ---
    public enum ExampleSubsystemPositions {
        Down(-90),
        Forwards(-90),
        Up(-90);

        final double angle;

        ExampleSubsystemPositions(double angle) {
            this.angle = angle;
        }

        public double get() {
                return angle;
            }
    }


    public static class Swerve {
        public static final double kDriverSpeedFactor = 1;
        public static final double kDriverRotationSpeedFactor = 1;
        public static final double kJoystickDeadband = 0.05;
        public static final boolean kDriverFieldRelative = false;

        public static final SwerveConstants kSwerveConstants = new SwerveConstants();
        static {
            // Move chassis-related settings to chassis subclass
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

            // Move limits-related settings to limits subclass
            kSwerveConstants.limits.maxSpeed = 4.5;
            kSwerveConstants.limits.maxAngularVelocity = 9.2;
            kSwerveConstants.limits.speedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationSpeedLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.accelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.rotationAccelerationLimit = Double.MAX_VALUE;
            kSwerveConstants.limits.maxSkidAcceleration = Double.MAX_VALUE;

            // Move modules-related settings to modules subclass
            kSwerveConstants.modules.openLoop = true;
            double wheelRadius = 0.048;
            kSwerveConstants.modules.driveMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.driveMotorConstants.real.currentLimit = 100;
            kSwerveConstants.modules.driveMotorConstants.real.gearRatio = 5.9;
            kSwerveConstants.modules.driveMotorConstants.real.conversionFactor = wheelRadius * 2 * Math.PI;
            kSwerveConstants.modules.driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(90,0.19);

            kSwerveConstants.modules.steerMotorConstants = new ControllerConstants();
            kSwerveConstants.modules.steerMotorConstants.real.currentLimit = 60;
            kSwerveConstants.modules.steerMotorConstants.real.gearRatio = 18.75;
            kSwerveConstants.modules.steerMotorConstants.real.conversionFactor = 2 * Math.PI;
            kSwerveConstants.modules.steerMotorConstants.real.controlConstants = ControlConstants.createPID(10,0,0,0);

            kSwerveConstants.modules.driveControllerType = Controller.ControllerType.TalonFX;
            kSwerveConstants.modules.steerControllerType = Controller.ControllerType.TalonFX;
            kSwerveConstants.modules.moduleConstants = new SwerveModuleConstants[4];

            for (int i = 0; i < 4; i++) {
                kSwerveConstants.modules.moduleConstants[i].moduleNumber = i;
                kSwerveConstants.modules.moduleConstants[i].driveMotorID = 10 + i * 2;
                kSwerveConstants.modules.moduleConstants[i].driveMotorInverted = false;
                kSwerveConstants.modules.moduleConstants[i].steerMotorID = 11 + i * 2;
                kSwerveConstants.modules.moduleConstants[i].steerMotorInverted = false;
                kSwerveConstants.modules.moduleConstants[i].canCoderID = 6 + i;
                kSwerveConstants.modules.moduleConstants[i].invertCANCoder = false;
            }

            kSwerveConstants.modules.moduleConstants[0].CANCoderOffset = -0.218750;
            kSwerveConstants.modules.moduleConstants[1].CANCoderOffset = 0.232422;
            kSwerveConstants.modules.moduleConstants[2].CANCoderOffset = 0.229248;
            kSwerveConstants.modules.moduleConstants[3].CANCoderOffset = 0.210938;

            // Move gyro-related settings to gyro subclass
            kSwerveConstants.gyro.gyroID = 45;
            kSwerveConstants.gyro.gyroInverted = false;
            kSwerveConstants.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

            // Move simulation-related settings to simulation subclass
            kSwerveConstants.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
            kSwerveConstants.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);

            // Move special-related settings to special subclass
            kSwerveConstants.special.enableOdometryThread = true;
            kSwerveConstants.special.odometryThreadFrequency = 250;
            kSwerveConstants.special.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kSwerveConstants.special.robotStartPose = new Pose2d(3, 3, Rotation2d.kZero);

            // Move CAN-related settings
            kSwerveConstants.modules.CANBus = "Swerve Bus";

            try {
                kSwerveConstants.special.robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }
        }

        public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
        static {
            kSwerveControllerConstants.swerveConstants = kSwerveConstants;
            kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(6, 0, 0.2, 0);
            kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(3, 0.5, 0.2, Units.degreesToRadians(15));
            kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
        }

        public static final PathFollowingController kAutonomyConfig =
            new PPHolonomicDriveController(
                    new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
                    new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
            );
    }

    public static class Vision {
        public static final double kMaxDistanceFilter = 3;
        public static final double kMaxSpeedFilter = 3;
        public static final double kMaxAngularSpeedFilter = 7;
        public static final double kMaxAmbiguityFilter = 0.2;
        public static final double kOdometryDriftPerMeter = 0.02;
        public static final double kCrashAcceleration = 15;
        public static final double kOdometryDriftPerCrash = 0.5;

        public static final VisionConstants kVisionConstants = new VisionConstants();
        static {
            kVisionConstants.cameras = Map.of(
        //            "FrontRight", Pair.of(new Transform3d(0.0815 + 0.1054, -0.0745, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(-7.5 - 1.5))), VisionConstants.CameraType.PhotonVision),
        //            "FrontLeft", Pair.of(new Transform3d(0.0815 + 0.1054, 0.0755, -0.191, new Rotation3d(0, 0, Units.degreesToRadians(7.5 - 1.5))), VisionConstants.CameraType.PhotonVision)
            "Right", Pair.of(new Transform3d(0.735 / 2, -0.03, 0, new Rotation3d(0, 0, 0)), VisionConstants.CameraType.PhotonVision)
        );

            kVisionConstants.fieldLayoutGetter = Constants.Field::getFieldLayoutWithIgnored;
            kVisionConstants.isReplay = Constants.General.kRobotMode == RobotMode.REPLAY;
            kVisionConstants.robotPoseSupplier = () -> RobotStateWithSwerve.getInstance().getRobotPose();
        }

        public static Matrix<N3, N1> getVisionSTD(VisionOutput output) {
            double distStd = Math.pow(0.8 * output.closestTargetDist, 2) + 0.3;

            ChassisSpeeds speed = frc.lib.NinjasLib.swerve.Swerve.getInstance().getChassisSpeeds(false);
            double xySpeedStd = 6 * output.latency * Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
            double angleSpeedStd = 6 * output.latency * speed.omegaRadiansPerSecond;

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

        static {
            try {
                kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
                kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

                kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
                kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
            } catch (IOException e) {
                throw new RuntimeException("Unable to load field layout");
            }
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
    }

    public static class AutoDrive {
        public static final double kAutoDriveDistFromReef = 0.5;
        public static final double kAutoDriveRightSideOffset = 0.25;
        public static final double kAutoDriveLeftSideOffset = 0.25;
        public static final double kAutoDriveDistThreshold = 0.3;
    }

}
