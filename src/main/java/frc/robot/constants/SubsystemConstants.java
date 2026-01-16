package frc.robot.constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    public static final ControllerConstants kClimberAngle = new ControllerConstants();
    static {
        /* Base */
        kClimberAngle.real.base.main.id = 41;
        kClimberAngle.real.base.main.inverted = false;
        kClimberAngle.real.base.currentLimit = 60;
        kClimberAngle.real.base.isBrakeMode = true;

        /* Control */
        kClimberAngle.real.control.controlConstants = ControlConstants.createProfiledPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        kClimberAngle.real.control.gearRatio = 1;
        kClimberAngle.real.control.conversionFactor = 1;
        kClimberAngle.real.control.positionGoalTolerance = Units.degreesToRadians(3);

        /* Soft Limits */
        kClimberAngle.real.softLimits.min = Units.degreesToRadians(0);
        kClimberAngle.real.softLimits.max = Units.degreesToRadians(90);

        /* Simulation */
        kClimberAngle.motorType = DCMotor.getKrakenX60(1);
    }

    public static final ControllerConstants kClimber = new ControllerConstants();
    static {
        /* Base */
        kClimber.real.base.main.id = 40;
        kClimber.real.base.main.inverted = false;
        kClimber.real.base.currentLimit = 60;
        kClimber.real.base.isBrakeMode = true;

        /* Control */
        kClimber.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kClimber.real.control.gearRatio = 2;

        /* Limit Switch */
        kClimber.real.hardLimit.id = 3;
        kClimber.real.hardLimit.homePosition = PositionsConstants.Climber.kLeftClimb.get();

        /* Simulation */
        kClimber.motorType = DCMotor.getKrakenX60(1);
    }

    public static final ControllerConstants kIntake = new ControllerConstants();
    static {
        /* Base */
        kIntake.real.base.main.id = 20;
        kIntake.real.base.main.inverted = false;
        kIntake.real.base.currentLimit = 60;
        kIntake.real.base.isBrakeMode = true;

        /* Control */
        kIntake.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kIntake.real.control.gearRatio = 2;

        /* Simulation */
        kIntake.motorType = DCMotor.getKrakenX60(1);
    }



    public static final ControllerConstants kIntakeAngle = new ControllerConstants();
    static {
        /* Base */
        kIntakeAngle.real.base.main.id = 21;
        kIntakeAngle.real.base.main.inverted = false;
        kIntakeAngle.real.base.currentLimit = 60;
        kIntakeAngle.real.base.isBrakeMode = true;

        /* Control */
        kIntakeAngle.real.control.controlConstants = ControlConstants.createProfiledPID(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, GravityTypeValue.Arm_Cosine);
        kIntakeAngle.real.control.gearRatio = 1;
        kIntakeAngle.real.control.conversionFactor = 1;
        kIntakeAngle.real.control.positionGoalTolerance = Units.degreesToRadians(3);

        /* Soft Limits */
        kIntakeAngle.real.softLimits.min = Units.degreesToRadians(-90);
        kIntakeAngle.real.softLimits.max = Units.degreesToRadians(90);

        /* CANCoder */
        kIntakeAngle.real.canCoder.enable = true;
        kIntakeAngle.real.canCoder.id = 23;
        kIntakeAngle.real.canCoder.mode = RealControllerConstants.CANCoder.CANCoderMode.Normal;
        kIntakeAngle.real.canCoder.config = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(0.5));

        /* Simulation */
        kIntakeAngle.motorType = DCMotor.getKrakenX60(1);
    }



    public static final ControllerConstants kIntakeIndexer = new ControllerConstants();
    static {
        /* Base */
        kIntakeIndexer.real.base.main.id = 22;
        kIntakeIndexer.real.base.main.inverted = false;
        kIntakeIndexer.real.base.currentLimit = 60;
        kIntakeIndexer.real.base.isBrakeMode = true;

        /* Control */
        kIntakeIndexer.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kIntakeIndexer.real.control.gearRatio = 2;

        /* Simulation */
        kIntakeIndexer.motorType = DCMotor.getKrakenX60(1);
    }


    public static final ControllerConstants kShooter = new ControllerConstants();
    static {
        /* Base */
        kShooter.real.base.main.id = 30;
        kShooter.real.base.main.inverted = false;
        kShooter.real.base.currentLimit = 80;
        kShooter.real.base.isBrakeMode = false;

        /* Control */
        kShooter.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kShooter.real.control.gearRatio = 1;
        kShooter.real.control.velocityGoalTolerance = 5;

        /* Simulation */
        kShooter.motorType = DCMotor.getKrakenX60(2);
    }



    public static final ControllerConstants kShooterIndexer = new ControllerConstants();
    static {
        /* Base */
        kShooterIndexer.real.base.main.id = 31;
        kShooterIndexer.real.base.main.inverted = false;
        kShooterIndexer.real.base.currentLimit = 80;
        kShooterIndexer.real.base.isBrakeMode = true;

        /* Control */
        kShooterIndexer.real.control.controlConstants = ControlConstants.createTorqueCurrent(10, 1, 0.2);
        kShooterIndexer.real.control.gearRatio = 2;

        /* Simulation */
        kShooterIndexer.motorType = DCMotor.getKrakenX60(2);
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
        kSwerve.limits.maxSpeed = 4.5;
        kSwerve.limits.maxAngularVelocity = 9.2;
        kSwerve.limits.speedLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationSpeedLimit = Double.MAX_VALUE;
        kSwerve.limits.accelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.rotationAccelerationLimit = Double.MAX_VALUE;
        kSwerve.limits.maxSkidAcceleration = 80;

        /* Modules */
        double wheelRadius = 0.048;
        kSwerve.modules.openLoop = false;
        kSwerve.modules.driveMotorConstants = new ControllerConstants();
        kSwerve.modules.driveMotorConstants.real.base.currentLimit = 100;
        kSwerve.modules.driveMotorConstants.real.control.gearRatio = 5.9;
        kSwerve.modules.driveMotorConstants.real.control.conversionFactor = wheelRadius * 2 * Math.PI;
        if (!GeneralConstants.kRobotMode.isSim())
            kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createTorqueCurrent(90, 1, 6);
        else
            kSwerve.modules.driveMotorConstants.real.control.controlConstants = ControlConstants.createTorqueCurrent(7, 0, 0);

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
        kSwerve.special.robotStartPose = new Pose2d(2, 4, Rotation2d.kZero);
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
            "Front", Pair.of(new Transform3d(0.28286, 0.12995, 0.152, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(-7.5))), VisionConstants.CameraType.Limelight),
            "Back", Pair.of(new Transform3d(-0.28286 - 0.05, 0.12995, 0.152, new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(187.5))), VisionConstants.CameraType.Limelight)
        );

        kVision.fieldLayoutGetter = FieldConstants::getFieldLayoutWithIgnored;
        kVision.isReplay = GeneralConstants.kRobotMode.isReplay();
        kVision.robotPoseSupplier = () -> RobotState.getInstance().getRobotPose();
    }
}
