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
import edu.wpi.first.math.util.Units;
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
        kIntake.real.base.currentLimit = 60;
        kIntake.real.base.isBrakeMode = false;

        /* Control */
        kIntake.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, 0, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kIntake.real.control.enableFOC = false;

        /* Simulation */
        kIntake.simMotor = DCMotor.getKrakenX60(1);
        kIntake.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kIntakeOpen = new ControllerConstants();
    static {
        /* Base */
        kIntakeOpen.real.base.main.id = 21;
        kIntakeOpen.real.base.main.inverted = true;
        kIntakeOpen.real.base.currentLimit = 60;
        kIntakeOpen.real.base.isBrakeMode = false;

        /* Control */
        kIntakeOpen.real.control.controlConstants = ControlConstants.createPID(120, 0, 0.03, 0);
        kIntakeOpen.real.control.conversionFactor = 0.00864;
        kIntakeOpen.real.control.positionGoalTolerance = 0.01;
        kIntakeOpen.real.control.enableFOC = false;

        /* Soft Limits */
        kIntakeOpen.real.softLimits.max = 0.3;

        /* Hard Limit */
        kIntakeOpen.real.hardLimits.limits = new RealControllerConstants.HardLimits.HardLimit[] { new RealControllerConstants.HardLimits.HardLimit(), new RealControllerConstants.HardLimits.HardLimit() };
        kIntakeOpen.real.hardLimits.limits[0].id = 0;
        kIntakeOpen.real.hardLimits.limits[0].inverted = true;
        kIntakeOpen.real.hardLimits.limits[0].direction = -1;
        kIntakeOpen.real.hardLimits.limits[0].autoStopReset = true;
        kIntakeOpen.real.hardLimits.limits[0].homePosition = 0;

        kIntakeOpen.real.hardLimits.limits[1].isVirtual = true;
        kIntakeOpen.real.hardLimits.limits[1].virtualStallThreshold = 18;
        kIntakeOpen.real.hardLimits.limits[1].virtualMinPos = 0.27;
        kIntakeOpen.real.hardLimits.limits[1].virtualFrames = 5;
        kIntakeOpen.real.hardLimits.limits[1].direction = 1;
        kIntakeOpen.real.hardLimits.limits[1].autoStopReset = true;
        kIntakeOpen.real.hardLimits.limits[1].homePosition = 0.3;

        /* Simulation */
        kIntakeOpen.simMotor = DCMotor.getKrakenX60(1);
        kIntakeOpen.simSystem = LinearSystemId.createElevatorSystem(kIntakeOpen.simMotor, 3, 0.02, kIntakeOpen.real.control.gearRatio);
    }



    public static final ControllerConstants kIndexer = new ControllerConstants();
    static {
        /* Base */
        kIndexer.real.base.main.id = 22;
        kIndexer.real.base.main.inverted = true;
        kIndexer.real.base.currentLimit = 60;
        kIndexer.real.base.isBrakeMode = false;

        /* Control */
        kIndexer.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, Double.POSITIVE_INFINITY, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kIndexer.real.control.enableFOC = false;

        /* Simulation */
        kIndexer.simMotor = DCMotor.getKrakenX60(1);
        kIndexer.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kIndexer2 = new ControllerConstants();
    static {
        /* Base */
        kIndexer2.real.base.main.id = 23;
        kIndexer2.real.base.main.inverted = false;
        kIndexer2.real.base.currentLimit = 60;
        kIndexer2.real.base.isBrakeMode = false;

        /* Control */
        kIndexer2.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, Double.POSITIVE_INFINITY, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kIndexer2.real.control.enableFOC = false;

        /* Simulation */
        kIndexer2.simMotor = DCMotor.getKrakenX60(1);
        kIndexer2.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
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
        kShooter.real.base.currentLimit = 60;
        kShooter.real.base.isBrakeMode = false;

        /* Control */
        kShooter.real.control.controlConstants = ControlConstants.createPIDF(0.5, 0, 0, Double.POSITIVE_INFINITY, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kShooter.real.control.velocityGoalTolerance = 6;
        kShooter.real.control.enableFOC = false;

        /* Simulation */
        kShooter.simMotor = DCMotor.getKrakenX60(2);
        kShooter.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (100 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kAccelerator = new ControllerConstants();
    static {
        /* Base */
        kAccelerator.real.base.main.id = 33;
        kAccelerator.real.base.main.inverted = true;
        kAccelerator.real.base.currentLimit = 60;
        kAccelerator.real.base.isBrakeMode = false;

        /* Control */
        kAccelerator.real.control.controlConstants = ControlConstants.createPIDF(0, 0, 0, Double.POSITIVE_INFINITY, 0.13, 0, 0, 0, GravityTypeValue.Elevator_Static);
        kAccelerator.real.control.velocityGoalTolerance = 10;
        kAccelerator.real.control.enableFOC = false;

        /* Simulation */
        kAccelerator.simMotor = DCMotor.getKrakenX60(1);
        kAccelerator.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (100 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kClimber = new ControllerConstants();
    static {
        /* Base */
        kClimber.real.base.main.id = 40;
        kClimber.real.base.currentLimit = 60;
        kClimber.real.base.followers = new SimpleControllerConstants[1];
        kClimber.real.base.followers[0].id = 41;

        /* Control */
        kClimber.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kClimber.real.control.gearRatio = 2;

        /* Limit Switch */
        kIntakeOpen.real.hardLimits.limits = new RealControllerConstants.HardLimits.HardLimit[] { new RealControllerConstants.HardLimits.HardLimit(), new RealControllerConstants.HardLimits.HardLimit() };
        kIntakeOpen.real.hardLimits.limits[0].id = 1;
        kIntakeOpen.real.hardLimits.limits[0].direction = -1;
        kIntakeOpen.real.hardLimits.limits[0].autoStopReset = true;
        kIntakeOpen.real.hardLimits.limits[0].homePosition = 0;

        /* Simulation */
        kClimber.simMotor = DCMotor.getKrakenX60(2);
        kClimber.simSystem = LinearSystemId.createElevatorSystem(kClimber.simMotor, 8, 0.04, kClimber.real.control.gearRatio);
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
        kSwerve.modules.driveMotorConstants.real.base.currentLimit = 100;
        kSwerve.modules.driveMotorConstants.real.control.gearRatio = 5.9;
        kSwerve.modules.driveMotorConstants.real.control.conversionFactor = wheelRadius * 2 * Math.PI;
//        kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createTorqueCurrent(60, 5, 3);
        kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createPIDF(1, 0, 0, 0.5, 2.35, 0.25, 0.3, 0, GravityTypeValue.Elevator_Static);

        kSwerve.modules.steerMotorConstants = new ControllerConstants();
        kSwerve.modules.steerMotorConstants.real.base.currentLimit = 60;
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
        kSwerve.special.enableOdometryThread = true;
        kSwerve.special.odometryThreadFrequency = 250;
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
            "limelight", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight)
        );

        kVision.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;
        kVision.isReplay = GeneralConstants.kRobotMode.isReplay();
        kVision.robotPoseSupplier = () -> RobotState.get().getRobotPose();
    }
}
