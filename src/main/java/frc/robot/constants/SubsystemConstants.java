package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.constants.ControlConstants;
import frc.lib.NinjasLib.controllers.constants.ControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants;
import frc.lib.NinjasLib.controllers.constants.RealControllerConstants.Base.SimpleControllerConstants;
import frc.lib.NinjasLib.localization.vision.VisionConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveControllerConstants;
import frc.lib.NinjasLib.swerve.constants.SwerveModuleConstants;
import frc.robot.RobotState;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Map;

public class SubsystemConstants {
    public static final ControllerConstants kIntake = new ControllerConstants();
    static {
        /* Base */
        kIntake.real.base.main.id = 20;
        kIntake.real.base.main.inverted = false;
        kIntake.real.base.isBrakeMode = false;
        kIntake.real.base.followers = new SimpleControllerConstants[] { new SimpleControllerConstants() };
        kIntake.real.base.followers[0].id = 21;
        kIntake.real.base.followers[0].inverted = true;

        /* Control */
        kIntake.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, 0, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kIntake.real.control.enableFOC = false;

        /* Simulation */
        kIntake.simMotor = DCMotor.getKrakenX60(1);
        kIntake.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kIntakeRail = new ControllerConstants();
    static {
        /* Base */
        kIntakeRail.real.base.main.id = 22;
        kIntakeRail.real.base.main.inverted = true;
        kIntakeRail.real.base.isBrakeMode = false;

        /* Control */
        kIntakeRail.real.control.controlConstants = ControlConstants.createPID(11, 4, 0.3, 1);
        kIntakeRail.real.control.conversionFactor = 1;//0.0089768737241178;
        kIntakeRail.real.control.positionGoalTolerance = 0.5;
        kIntakeRail.real.control.enableFOC = false;

        /* Soft Limits */

        /* Hard Limit */
        kIntakeRail.real.hardLimits.limits = new RealControllerConstants.HardLimits.HardLimit[] { new RealControllerConstants.HardLimits.HardLimit(), new RealControllerConstants.HardLimits.HardLimit() };
        kIntakeRail.real.hardLimits.limits[0].id = 0;
        kIntakeRail.real.hardLimits.limits[0].inverted = true;
        kIntakeRail.real.hardLimits.limits[0].direction = -1;
        kIntakeRail.real.hardLimits.limits[0].autoStopReset = true;
        kIntakeRail.real.hardLimits.limits[0].homePosition = 0;

        kIntakeRail.real.hardLimits.limits[1].isVirtual = true;
        kIntakeRail.real.hardLimits.limits[1].virtualStallThreshold = 25;
        kIntakeRail.real.hardLimits.limits[1].minPos = 31;
        kIntakeRail.real.hardLimits.limits[1].frames = 12;
        kIntakeRail.real.hardLimits.limits[1].direction = 1;
        kIntakeRail.real.hardLimits.limits[1].autoStopReset = true;
        kIntakeRail.real.hardLimits.limits[1].homePosition = 34.6;

        /* Simulation */
        kIntakeRail.simMotor = DCMotor.getKrakenX60(1);
        kIntakeRail.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (1000 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kBox = new ControllerConstants();
    static {
        /* Base */
        kBox.real.base.main.id = 50;
        kBox.real.base.main.inverted = false;
        kBox.real.base.isBrakeMode = false;
//        kBox.real.base.followers = new SimpleControllerConstants[] { new SimpleControllerConstants() };
//        kBox.real.base.followers[0].id = 51;
//        kBox.real.base.followers[0].inverted = true;

        /* Control */
        kBox.real.control.controlConstants = ControlConstants.createPID(5, 2, 0.1, 3);
        kBox.real.control.conversionFactor = 1;
        kBox.real.control.positionGoalTolerance = 0.5;
        kBox.real.control.enableFOC = false;

        /* Hard Limit */
//        kBox.real.hardLimits.limits = new RealControllerConstants.HardLimits.HardLimit[] { new RealControllerConstants.HardLimits.HardLimit() };
//        kBox.real.hardLimits.limits[0].id = 1;
//        kBox.real.hardLimits.limits[0].direction = -1;
//        kBox.real.hardLimits.limits[0].autoStopReset = true;
//        kBox.real.hardLimits.limits[0].homePosition = 0;

        /* Simulation */
        kBox.simMotor = DCMotor.getKrakenX60(1);
        kBox.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (1000 * 2 * Math.PI / 0.5));
    }

    public static ControllerConstants kRightBox = kBox.clone();
    static {
        kRightBox.real.base.main.id = 51;
        kRightBox.real.base.main.inverted = true;
    }



    public static final ControllerConstants kIndexer = new ControllerConstants();
    static {
        /* Base */
        kIndexer.real.base.main.id = 40;
        kIndexer.real.base.main.inverted = true;
        kIndexer.real.base.isBrakeMode = false;

        kIndexer.real.base.followers = new SimpleControllerConstants[] { new SimpleControllerConstants() };
        kIndexer.real.base.followers[0].id = 41;
        kIndexer.real.base.followers[0].inverted = true;

        /* Control */
        kIndexer.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, Double.POSITIVE_INFINITY, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kIndexer.real.control.enableFOC = true;

        /* Simulation */
        kIndexer.simMotor = DCMotor.getKrakenX60(1);
        kIndexer.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (500 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kShooter = new ControllerConstants();
    static {
        /* Base */
        kShooter.real.base.main.id = 30;
        kShooter.real.base.main.inverted = true;
        kShooter.real.base.followers = new SimpleControllerConstants[] { new SimpleControllerConstants(), new SimpleControllerConstants() };
        kShooter.real.base.followers[0].id = 31;
        kShooter.real.base.followers[0].inverted = false;
        kShooter.real.base.followers[1].id = 32;
        kShooter.real.base.followers[1].inverted = false;
        kShooter.real.base.isBrakeMode = false;

        /* Control */
        kShooter.real.control.controlConstants = ControlConstants.createPIDF(0.5, 1, 0, Double.POSITIVE_INFINITY, 0.1175, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kShooter.real.control.velocityGoalTolerance = 6;
        kShooter.real.control.enableFOC = false;

        /* Simulation */
        kShooter.simMotor = DCMotor.getKrakenX60(2);
        kShooter.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (1000 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kAccelerator = new ControllerConstants();
    static {
        /* Base */
        kAccelerator.real.base.main.id = 33;
        kAccelerator.real.base.main.inverted = true;
        kAccelerator.real.base.isBrakeMode = false;

        /* Control */
        kAccelerator.real.control.controlConstants = ControlConstants.createPIDF(0, 0, 0, Double.POSITIVE_INFINITY, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kAccelerator.real.control.velocityGoalTolerance = 10;
        kAccelerator.real.control.enableFOC = false;

        /* Simulation */
        kAccelerator.simMotor = DCMotor.getKrakenX60(1);
        kAccelerator.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (1000 * 2 * Math.PI / 0.5));
    }



    public static final SwerveConstants kSwerve = new SwerveConstants();
    static {
        /* Chassis */
        kSwerve.chassis.trackWidth = 0.59475;
        kSwerve.chassis.wheelBase = 0.51855;
        kSwerve.chassis.bumperLength = 0.8079;
        kSwerve.chassis.bumperWidth = 0.8841;
        kSwerve.chassis.kinematics = new SwerveDriveKinematics(
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0)
        );

        /* Limits */
        kSwerve.limits.maxSpeed = GeneralConstants.kRobotMode.isSim() ? 5.145 : 4.7;
        kSwerve.limits.maxAngularVelocity = 8.5;
        kSwerve.limits.speedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationSpeedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationAccelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.maxSkidAcceleration = 60;
        kSwerve.limits.maxForwardAcceleration = 20;
        kSwerve.limits.discretizeFactor = 2.75;

        /* Modules */
        double wheelRadius = 0.049;
        kSwerve.modules.openLoop = GeneralConstants.kRobotMode.isSim();
        kSwerve.modules.driveMotorConstants = new ControllerConstants();
        kSwerve.modules.driveMotorConstants.real.base.statorCurrentLimit = 100;
        kSwerve.modules.driveMotorConstants.real.base.supplyCurrentLimit = 60;
        kSwerve.modules.driveMotorConstants.real.control.gearRatio = 5.9;
        kSwerve.modules.driveMotorConstants.real.control.conversionFactor = wheelRadius * 2 * Math.PI;
        kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createPIDF(1, 0, 0, 0.5, 2.35, 0, 0.3, 0, GravityTypeValue.Elevator_Static);

        kSwerve.modules.steerMotorConstants = new ControllerConstants();
        kSwerve.modules.steerMotorConstants.real.base.statorCurrentLimit = 40;
        kSwerve.modules.steerMotorConstants.real.base.supplyCurrentLimit = 30;
        kSwerve.modules.steerMotorConstants.real.control.gearRatio = 18.75;
        kSwerve.modules.steerMotorConstants.real.control.conversionFactor = 2 * Math.PI;
        kSwerve.modules.steerMotorConstants.real.control.controlConstants = ControlConstants.createPID(25, 30, 0.25, Math.PI);

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

        kSwerve.modules.moduleConstants[0].CANCoderOffset = -0.294189;
        kSwerve.modules.moduleConstants[1].CANCoderOffset = -0.472412;
        kSwerve.modules.moduleConstants[2].CANCoderOffset = -0.274170;
        kSwerve.modules.moduleConstants[3].CANCoderOffset = 0.481934;

        /* Gyro */
        kSwerve.gyro.gyroID = 5;
        kSwerve.gyro.gyroInverted = false;
        kSwerve.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

        /* Simulation */
        kSwerve.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerve.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerve.simulation.swerveType = SwerveConstants.Simulation.SwerveType.Mark4n;
        kSwerve.simulation.gearRatioLevel = 2;

        /* Special */
        kSwerve.special.enableOdometryThread = false;
        kSwerve.special.odometryThreadFrequency = 50;
        kSwerve.special.isReplay = GeneralConstants.kRobotMode.isReplay();
        kSwerve.special.robotStartPose = new Pose2d(2, 4, Rotation2d.kZero);
        kSwerve.special.CANBus = new CANBus("Swerve Bus");

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
        kSwerveController.rotationPIDConstants = ControlConstants.createPID(5.5, 0, 0, 0);
        kSwerveController.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
    }



    public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
        new PIDConstants(3.5, 0, 0),
        new PIDConstants(kSwerveController.rotationPIDConstants.P, kSwerveController.rotationPIDConstants.I, kSwerveController.rotationPIDConstants.D)
    );



    public static final VisionConstants kVision = new VisionConstants();
    static {
        kVision.cameras = Map.of(
            "limelight-front", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight),
            "limelight-right", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight)
        );

        kVision.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;
        kVision.isReplay = GeneralConstants.kRobotMode.isReplay();
        kVision.robotPoseSupplier = () -> RobotState.get().getRobotPose();
    }
}
