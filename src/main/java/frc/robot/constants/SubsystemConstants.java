package frc.robot.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import frc.robot.RobotState;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Map;

public class SubsystemConstants {
    public static final ControllerConstants kArm = new ControllerConstants();
    static {
        /* Base */
        kArm.real.main.id = 40;
        kArm.real.main.inverted = true;
        kArm.real.currentLimit = 60;
        kArm.real.isBrakeMode = true;

        /* Control */
        kArm.real.controlConstants = ControlConstants.createProfiledPID(120, 0, 0, 0, 2, 2, 0, 0, 0, 0.3, GravityTypeValue.Arm_Cosine);
        kArm.real.gearRatio = 86.4;
        kArm.real.homePosition = Units.degreesToRotations(-90);
        kArm.real.positionGoalTolerance = Units.degreesToRotations(4);

        /* Soft Limits */
        kArm.real.maxSoftLimit = Units.degreesToRotations(360);
        kArm.real.minSoftLimit = Units.degreesToRotations(-360);

        /* Simulation */
        kArm.motorType = DCMotor.getKrakenX60(1);
    }



    public static final ControllerConstants kElevator = new ControllerConstants();
    static {
        /* Base */
        kElevator.real.main.id = 30;
        kElevator.real.main.inverted = false;
        kElevator.real.currentLimit = 60;
        kElevator.real.isBrakeMode = true;

        /* Followers */
        kElevator.real.followers = new RealControllerConstants.SimpleControllerConstants[1];
        kElevator.real.followers[0] = new RealControllerConstants.SimpleControllerConstants();
        kElevator.real.followers[0].id = 31;
        kElevator.real.followers[0].inverted = true;

        /* Control */
        kElevator.real.controlConstants = ControlConstants.createProfiledPID(20, 0, 0, 0, 14, 100, 0, 0.7, 0.3, 0.3, GravityTypeValue.Elevator_Static);
        kElevator.real.gearRatio = 6;
        kElevator.real.homePosition = 0;
        kElevator.real.positionGoalTolerance = 0.05;

        /* Soft Limits */
        kElevator.real.maxSoftLimit = 10.8;

        /* Hard Limit */
        kElevator.real.isLimitSwitch = true;
        kElevator.real.limitSwitchID = 7;
        kElevator.real.limitSwitchDirection = -1;
        kElevator.real.limitSwitchAutoStopReset = true;
        kElevator.real.limitSwitchInverted = true;

        /* Simulation */
        kElevator.motorType = DCMotor.getKrakenX60(2);
    }



    public static final ControllerConstants kOuttake = new ControllerConstants();
    static {
        /* Base */
        kOuttake.real.main.id = 50;
        kOuttake.real.main.inverted = false;
        kOuttake.real.currentLimit = 60;
        kOuttake.real.isBrakeMode = true;

        /* Simulation */
        kOuttake.motorType = DCMotor.getKrakenX60(1);
    }



    public static final ControllerConstants kIntake = new ControllerConstants();
    static {
        /* Base */
        kIntake.real.main.id = 20;
        kIntake.real.main.inverted = false;
        kIntake.real.currentLimit = 80;
        kIntake.real.isBrakeMode = true;

        /* Control */
        kIntake.real.controlConstants = ControlConstants.createTorqueCurrent(20, 0);

        /* Simulation */
        kIntake.motorType = DCMotor.getKrakenX60(1);
    }



    public static final ControllerConstants kIntakeAngle = new ControllerConstants();
    static {
        /* Base */
        kIntakeAngle.real.main.id = 21;
        kIntakeAngle.real.main.inverted = true;
        kIntakeAngle.real.currentLimit = 50;
        kIntakeAngle.real.isBrakeMode = true;

        /* Control */
        kIntakeAngle.real.controlConstants = ControlConstants.createProfiledPID(40, 0, 0, 0, 7, 35, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        kIntakeAngle.real.gearRatio = 65 + 1 / 3.0;
        kIntakeAngle.real.conversionFactor = 2 * Math.PI;
        kIntakeAngle.real.homePosition = Units.degreesToRadians(0);
        kIntakeAngle.real.positionGoalTolerance = Units.degreesToRadians(3);

        /* Simulation */
        kIntakeAngle.motorType = DCMotor.getKrakenX60(1);
    }



    public static final ControllerConstants kIntakeAligner = new ControllerConstants();
    static {
        /* Base */
        kIntakeAligner.real.main.id = 22;
        kIntakeAligner.real.main.inverted = true;
        kIntakeAligner.real.currentLimit = 80;
        kIntakeAligner.real.isBrakeMode = true;

        /* Control */
        kIntakeAligner.real.controlConstants = ControlConstants.createTorqueCurrent(20, 0);

        /* Simulation */
        kIntakeAligner.motorType = DCMotor.getKrakenX60(1);
    }



    public static final SwerveConstants kSwerve = new SwerveConstants();
    static {
        /* Chassis */
        kSwerve.chassis.trackWidth = 0.735;
        kSwerve.chassis.wheelBase = 0.735;
        kSwerve.chassis.bumperLength = 0.896;
        kSwerve.chassis.bumperWidth = 0.896;
        kSwerve.chassis.kinematics = new SwerveDriveKinematics(
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0)
        );

        /* Limits */
        kSwerve.limits.maxSpeed = 4;
        kSwerve.limits.maxAngularVelocity = 9.2;
        kSwerve.limits.speedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationSpeedLimit = Double.MAX_VALUE;
        kSwerve.limits.accelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationAccelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.maxSkidAcceleration = 65;

        /* Modules */
        double wheelRadius = 0.048;
        kSwerve.modules.openLoop = false;
        kSwerve.modules.driveMotorConstants = new ControllerConstants();
        kSwerve.modules.driveMotorConstants.real.currentLimit = 100;
        kSwerve.modules.driveMotorConstants.real.gearRatio = 5.9;
        kSwerve.modules.driveMotorConstants.real.conversionFactor = wheelRadius * 2 * Math.PI;
        if (!GeneralConstants.kRobotMode.isSim())
            kSwerve.modules.driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(90, 6);
        else
            kSwerve.modules.driveMotorConstants.real.controlConstants = ControlConstants.createTorqueCurrent(7, 0);

        kSwerve.modules.steerMotorConstants = new ControllerConstants();
        kSwerve.modules.steerMotorConstants.real.currentLimit = 60;
        kSwerve.modules.steerMotorConstants.real.gearRatio = 18.75;
        kSwerve.modules.steerMotorConstants.real.conversionFactor = 2 * Math.PI;
        kSwerve.modules.steerMotorConstants.real.controlConstants = ControlConstants.createPID(10, 0, 0, 0);

        kSwerve.modules.driveControllerType = Controller.ControllerType.TalonFX;
        kSwerve.modules.steerControllerType = Controller.ControllerType.TalonFX;
        kSwerve.modules.moduleConstants = new SwerveModuleConstants[4];

        for (int i = 0; i < 4; i++) {
            kSwerve.modules.moduleConstants[i] = new SwerveModuleConstants(
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

        kSwerve.modules.moduleConstants[0].CANCoderOffset = -0.291260;
        kSwerve.modules.moduleConstants[1].CANCoderOffset = -0.268799;
        kSwerve.modules.moduleConstants[2].CANCoderOffset = -0.267334;
        kSwerve.modules.moduleConstants[3].CANCoderOffset = 0.279541;

        /* Gyro */
        kSwerve.gyro.gyroID = 45;
        kSwerve.gyro.gyroInverted = false;
        kSwerve.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

        /* Simulation */
        kSwerve.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerve.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);

        /* Special */
        kSwerve.special.enableOdometryThread = true;
        kSwerve.special.odometryThreadFrequency = 250;
        kSwerve.special.isReplay = GeneralConstants.kRobotMode.isReplay();
        kSwerve.special.robotStartPose = new Pose2d(3, 3, Rotation2d.kZero);
        kSwerve.special.CANBus = "Swerve Bus";

        try {
            kSwerve.special.robotConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException(e);
        }
    }



    public static final SwerveControllerConstants kSwerveController = new SwerveControllerConstants();
    static {
        kSwerveController.swerveConstants = kSwerve;
        kSwerveController.drivePIDConstants = ControlConstants.createPID(5, 0, 0, 0);
        kSwerveController.rotationPIDConstants = ControlConstants.createPID(6, 0.5, 0, Units.degreesToRadians(5));
        kSwerveController.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
    }



    public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
        new PIDConstants(kSwerveController.drivePIDConstants.P, kSwerveController.drivePIDConstants.I, kSwerveController.drivePIDConstants.D),
        new PIDConstants(kSwerveController.rotationPIDConstants.P, kSwerveController.rotationPIDConstants.I, kSwerveController.rotationPIDConstants.D)
    );



    public static final VisionConstants kVision = new VisionConstants();
    static {
        kVision.cameras = Map.of(
            "Front", Pair.of(new Transform3d(0.28286, 0.12995, 0.152, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-7.5))), VisionConstants.CameraType.PhotonVision),
            "Back", Pair.of(new Transform3d(-0.28286 - 0.05, 0.12995, 0.152, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(187.5))), VisionConstants.CameraType.PhotonVision)
        );

        kVision.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;
        kVision.isReplay = GeneralConstants.kRobotMode.isReplay();
        kVision.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
    }

    public static class Other {
        public static class CANCoderConstants {
            public final int canCoderID;
            public final double canCoderOffset;
            public final SensorDirectionValue canCoderReversed;

            public CANCoderConstants(int canCoderID, double canCoderOffset, SensorDirectionValue canCoderReversed) {
                this.canCoderID = canCoderID;
                this.canCoderReversed = canCoderReversed;
                this.canCoderOffset = canCoderOffset;
            }
        }

        public static final CANCoderConstants kArmCANCoder = new CANCoderConstants(42, 1.010498 - Units.degreesToRotations(2), SensorDirectionValue.CounterClockwise_Positive);

        public static final CANCoderConstants kIntakeAngleCANCoder = new CANCoderConstants(23, 1.322266 - 0.268311, SensorDirectionValue.CounterClockwise_Positive);

        public static final int kIntakeBeamBreakerPort = 8;
    }
}
