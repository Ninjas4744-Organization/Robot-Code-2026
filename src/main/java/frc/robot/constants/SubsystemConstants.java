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

        /* Control */
         kIntake.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kIntake.real.control.gearRatio = 2;

        /* Simulation */
        kIntake.simMotor = DCMotor.getKrakenX60(1);
        kIntake.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kIntakeOpen = new ControllerConstants();
    static {
        /* Base */
        kIntakeOpen.real.base.main.id = 21;
        kIntakeOpen.real.base.currentLimit = 60;

        /* Control */
        kIntakeOpen.real.control.controlConstants = ControlConstants.createProfiledPID(1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        kIntakeOpen.real.control.positionGoalTolerance = 0.02;

        /* Soft Limits */
        kIntakeOpen.real.softLimits.max = 0.5;

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

        /* Control */
        kIndexer.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kIndexer.real.control.gearRatio = 2;

        /* Simulation */
        kIndexer.simMotor = DCMotor.getKrakenX60(1);
        kIndexer.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kIndexer2 = new ControllerConstants();
    static {
        /* Base */
        kIndexer2.real.base.main.id = 23;
        kIndexer2.real.base.main.inverted = true;
        kIndexer2.real.base.currentLimit = 60;

        /* Control */
        kIndexer2.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kIndexer2.real.control.gearRatio = 2;

        /* Simulation */
        kIndexer2.simMotor = DCMotor.getKrakenX60(1);
        kIndexer2.simSystem = LinearSystemId.createDCMotorSystem(12 / (50 * 2 * Math.PI), 12 / (50 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kShooter = new ControllerConstants();
    static {
        /* Base */
        kShooter.real.base.main.id = 30;
        kShooter.real.base.main.inverted = true;
        kShooter.real.base.followers = new SimpleControllerConstants[] { new SimpleControllerConstants() };
        kShooter.real.base.followers[0].id = 31;
        kShooter.real.base.followers[0].inverted = true;
        kShooter.real.base.currentLimit = 90;
        kShooter.real.base.isBrakeMode = false;

        /* Control */
        kShooter.real.control.controlConstants = ControlConstants.createPID(0, 0, 0,Double.POSITIVE_INFINITY);
        kShooter.real.control.controlConstants.V = 0.125;
        kShooter.real.control.velocityGoalTolerance = 3;

        /* Simulation */
        kShooter.simMotor = DCMotor.getKrakenX60(2);
        kShooter.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (100 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kAccelerator = new ControllerConstants();
    static {
        /* Base */
        kAccelerator.real.base.main.id = 32;
        kAccelerator.real.base.main.inverted = true;
        kAccelerator.real.base.currentLimit = 80;
        kAccelerator.real.base.isBrakeMode = false;

        /* Control */
        kAccelerator.real.control.controlConstants = ControlConstants.createPID(0.4, 0, 0, 0);
        kAccelerator.real.control.controlConstants.V = 0.13;
        kAccelerator.real.control.velocityGoalTolerance = 10;

        /* Simulation */
        kAccelerator.simMotor = DCMotor.getKrakenX60(1);
        kAccelerator.simSystem = LinearSystemId.createDCMotorSystem(12 / (100 * 2 * Math.PI), 12 / (100 * 2 * Math.PI / 0.5));
    }



    public static final ControllerConstants kClimber = new ControllerConstants();
    static {
        /* Base */
        kClimber.real.base.main.id = 40;
        kClimber.real.base.currentLimit = 60;

        /* Control */
        kClimber.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kClimber.real.control.gearRatio = 2;

        /* Limit Switch */
        kClimber.real.hardLimit.id = 3;
        kClimber.real.hardLimit.homePosition = PositionsConstants.Climber.kLeftClimb.get();

        /* Simulation */
        kClimber.simMotor = DCMotor.getKrakenX60(2);
        kClimber.simSystem = LinearSystemId.createElevatorSystem(kClimber.simMotor, 8, 0.04, kClimber.real.control.gearRatio);
    }



    public static final ControllerConstants kClimberAngle = new ControllerConstants();
    static {
        /* Base */
        kClimberAngle.real.base.main.id = 41;
        kClimberAngle.real.base.currentLimit = 60;

        /* Control */
        kClimberAngle.real.control.controlConstants = ControlConstants.createProfiledPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        kClimberAngle.real.control.gearRatio = 1;
        kClimberAngle.real.control.conversionFactor = 1;
        kClimberAngle.real.control.positionGoalTolerance = Units.degreesToRadians(3);

        /* Soft Limits */
        kClimberAngle.real.softLimits.min = Units.degreesToRadians(0);
        kClimberAngle.real.softLimits.max = Units.degreesToRadians(90);

        /* Simulation */
        kClimberAngle.simMotor = DCMotor.getKrakenX60(2);
        kClimberAngle.simSystem = LinearSystemId.createSingleJointedArmSystem(kClimberAngle.simMotor, 1, kClimberAngle.real.control.gearRatio);
    }



    public static final SwerveConstants kSwerve = new SwerveConstants();
    static {
        /* Chassis */
        kSwerve.chassis.trackWidth = 0.59475;
        kSwerve.chassis.wheelBase = 0.51855;
        kSwerve.chassis.bumperLength = 0.8709;
        kSwerve.chassis.bumperWidth = 0.8841;
        kSwerve.chassis.kinematics = new SwerveDriveKinematics(
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, kSwerve.chassis.trackWidth / 2.0),
            new Translation2d(-kSwerve.chassis.wheelBase / 2.0, -kSwerve.chassis.trackWidth / 2.0)
        );

        /* Limits */
        kSwerve.limits.maxSpeed = 4.42;
        kSwerve.limits.maxAngularVelocity = 8.5;
        kSwerve.limits.speedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationSpeedLimit = Double.MAX_VALUE;
        kSwerve.limits.accelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationAccelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.maxSkidAcceleration = 7.5;
        kSwerve.limits.maxForwardAcceleration = 10;

        /* Modules */
        double wheelRadius = 0.048;
        kSwerve.modules.openLoop = GeneralConstants.kRobotMode.isSim();
        kSwerve.modules.driveMotorConstants = new ControllerConstants();
        kSwerve.modules.driveMotorConstants.real.base.currentLimit = 100;
        kSwerve.modules.driveMotorConstants.real.control.gearRatio = 5.36;
        kSwerve.modules.driveMotorConstants.real.control.conversionFactor = wheelRadius * 2 * Math.PI;
        kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createTorqueCurrent(60, 5, 3);

        kSwerve.modules.steerMotorConstants = new ControllerConstants();
        kSwerve.modules.steerMotorConstants.real.base.currentLimit = 60;
        kSwerve.modules.steerMotorConstants.real.control.gearRatio = 18.75;
        kSwerve.modules.steerMotorConstants.real.control.conversionFactor = 2 * Math.PI;
        kSwerve.modules.steerMotorConstants.real.control.controlConstants = ControlConstants.createPID(10, 0, 0, 0);

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
        kSwerve.modules.moduleConstants[1].CANCoderOffset = -0.482422;
        kSwerve.modules.moduleConstants[2].CANCoderOffset = -0.264160;
        kSwerve.modules.moduleConstants[3].CANCoderOffset = 0.475342;

        /* Gyro */
        kSwerve.gyro.gyroID = 5;
        kSwerve.gyro.gyroInverted = false;
        kSwerve.gyro.gyroType = SwerveConstants.Gyro.GyroType.Pigeon2;

        /* Simulation */
        kSwerve.simulation.driveMotorType = DCMotor.getKrakenX60Foc(1);
        kSwerve.simulation.steerMotorType = DCMotor.getKrakenX60Foc(1);

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
        kSwerveController.rotationPIDConstants = ControlConstants.createPID(6.5, 0, 0, 0);
        kSwerveController.rotationPIDContinuousConnections = Pair.of(-Math.PI, Math.PI);
    }



    public static final PathFollowingController kAutonomyConfig = new PPHolonomicDriveController(
        new PIDConstants(kSwerveController.drivePIDConstants.P, kSwerveController.drivePIDConstants.I, kSwerveController.drivePIDConstants.D),
        new PIDConstants(kSwerveController.rotationPIDConstants.P, kSwerveController.rotationPIDConstants.I, kSwerveController.rotationPIDConstants.D)
    );



    public static final VisionConstants kVision = new VisionConstants();
    static {
        kVision.cameras = Map.of(
            "limelight", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight)
        );

        kVision.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;
        kVision.isReplay = GeneralConstants.kRobotMode.isReplay();
        kVision.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
    }
}
