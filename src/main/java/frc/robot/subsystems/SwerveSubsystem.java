package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.DerivativeCalculator2d;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.RobotState;
import frc.robot.ShootCalculator;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.SwerveSubsystem.SwerveState.*;

public class SwerveSubsystem extends StateMachineBase<SwerveSubsystem.SwerveState> {
    public enum SwerveState {
        UNKNOWN,
        DRIVER,
        AUTO,
        LOOK_HUB,
        DELIVERY,
        SNAP_ANGLE,
        AUTO_TRENCH,
    }

    private DoubleSupplier driverLeftX, driverLeftY, driverRightX, driverRightY;
    private SwerveSpeeds autoInput;
    private Pose2d target = new Pose2d();
    private DerivativeCalculator2d accelerationCalculator = new DerivativeCalculator2d(1);
    private boolean enabled;

    private Rotation2d lastLookHubTargetAngle = new Rotation2d();
    private double lookHubFF = 0;
    private static final double autoTrenchMaxStrength = 1;
    private static final double autoTrenchExp = 0.75;

    public SwerveSubsystem(boolean enabled, boolean enabledMinimum, DoubleSupplier driverLeftX, DoubleSupplier driverLeftY, DoubleSupplier driverRightX, DoubleSupplier driverRightY) {
        super(SwerveState.class);
        currentState = UNKNOWN;

        this.enabled = enabled;
        this.driverLeftX = driverLeftX;
        this.driverLeftY = driverLeftY;
        this.driverRightX = driverRightX;
        this.driverRightY = driverRightY;

        if (enabled) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
            SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
            SwerveController.getInstance().setChannel("Driver");
        } else if (enabledMinimum) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        }
    }

    @Override
    protected void define() {
        addStateCommand(DRIVER, Commands.run(() -> {
            SwerveController.getInstance().setControl(getDriveInput(), "Driver");
        }));

        addStateCommand(AUTO, Commands.run(() -> {
            SwerveController.getInstance().setControl(getDriveInput(), "Auto");
        }));

        addStateCommand(LOOK_HUB, Commands.sequence(
            Commands.runOnce(() -> {
                lastLookHubTargetAngle = ShootCalculator.getShootParams().angle();
                slowForShoot();
                SwerveController.getInstance().setChannel("Look Hub");
            }),
            Commands.run(() -> {
                target = new Pose2d(RobotState.get().getTranslation(), ShootCalculator.getShootParams().angle());

                double FF = target.getRotation().minus(lastLookHubTargetAngle).div(0.02).times(lookHubFF).getRadians();
                Logger.recordOutput("Robot/Shooting/Virtual Target Rotational Velocity", target.getRotation().minus(lastLookHubTargetAngle).div(0.02).getRadians());
                Logger.recordOutput("Robot/Shooting/Look Hub FF", FF);
                lastLookHubTargetAngle = target.getRotation();

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond,
                    SwerveController.getInstance().lookAt(target.getRotation()) + FF,
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Look Hub");
            })
        ).finallyDo(this::unSlow));

        addStateCommand(DELIVERY, Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Delivery");
            }),
            Commands.run(() -> {
                Rotation2d angleToDelivery = ShootCalculator.getShootParams().angle();
                target = new Pose2d(RobotState.get().getRobotPose().getX(), RobotState.get().getRobotPose().getY(), angleToDelivery);

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Delivery");
            })
        ));

        addStateCommand(SNAP_ANGLE, Commands.sequence(
            Commands.runOnce(() -> {
                target = RobotState.get().getRobotPose();
                target = new Pose2d(target.getX(), target.getY(), Rotation2d.fromDegrees(Math.round(target.getRotation().getDegrees() / 90.0) * 90.0));
                SwerveController.getInstance().setChannel("Snap Angle");
            }),
            Commands.run(() -> {
                target = new Pose2d(RobotState.get().getRobotPose().getX(),
                    RobotState.get().getRobotPose().getY(),
                    Rotation2d.fromDegrees(Math.round(target.getRotation().getDegrees() / 90.0) * 90.0));

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Snap Angle");
            })
        ));

        addStateCommand(AUTO_TRENCH, Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Auto Trench");
            }),
            Commands.run(() -> {
                if (!FieldConstants.nearLeftTrench() && !FieldConstants.nearRightTrench())
                    return;

                boolean isRightTrench = FieldConstants.nearRightTrench();

                target = isRightTrench ? FieldConstants.getRightTrenchPose() : FieldConstants.getLeftTrenchPose();
                target = new Pose2d(
                    RobotState.get().getRobotPose().getX(),
                    target.getY(),
                    Rotation2d.fromDegrees(Math.round(RobotState.get().getRobotPose().getRotation().getDegrees() / 90) * 90)
                );

                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                if (!getDriveInput().fieldRelative) {
                    ChassisSpeeds pidRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(pid.getX(), pid.getY(), 0), RobotState.get().getRobotPose().getRotation());
                    pid = new Translation2d(pidRobotRelative.vxMetersPerSecond, pidRobotRelative.vyMetersPerSecond);
                }

                double dist = FieldConstants.nearRightTrench()
                    ? RobotState.get().getDistance(FieldConstants.getRightTrenchPose())
                    : RobotState.get().getDistance(FieldConstants.getLeftTrenchPose());
                double strength = Math.pow(Math.pow(autoTrenchMaxStrength, 1 / autoTrenchExp) - Math.pow(autoTrenchMaxStrength, 1 / autoTrenchExp) * (dist / PositionsConstants.Swerve.kAutoTrenchThreshold.get()), autoTrenchExp);
                strength = MathUtil.clamp(strength, 0, 1);
                Logger.recordOutput("Robot/Swerve/Trench Strength", strength);

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond * (1 - strength) + pid.getY() * strength,
                    getDriveInput().omegaRadiansPerSecond * (1 - strength) + SwerveController.getInstance().lookAt(target.getRotation()) * strength,
                    getDriveInput().fieldRelative
                ), "Auto Trench");
            })
        ));

        addOmniEdge(DRIVER, Commands::none);
        addOmniEdge(AUTO, Commands::none);
        addOmniEdge(LOOK_HUB, Commands::none);
        addOmniEdge(DELIVERY, Commands::none);
        addOmniEdge(SNAP_ANGLE, Commands::none);
        addOmniEdge(AUTO_TRENCH, Commands::none);
    }

    public boolean atGoal() {
        boolean atPos = RobotState.get().getDistance(target) < PositionsConstants.Swerve.kPositionThreshold.get();
        boolean atAngleSmart = Math.abs(target.getRotation().minus(RobotState.get().getRobotPose().getRotation()).getDegrees()) < (PositionsConstants.Swerve.kAngleThresholdBase.get() + PositionsConstants.Swerve.kAngleThresholdCoefficient.get() * FieldConstants.getDistToHub().get());
        boolean atAngle = Math.abs(target.getRotation().minus(RobotState.get().getRobotPose().getRotation()).getDegrees()) < PositionsConstants.Swerve.kAngleThreshold.get();
        boolean atAcceleration = getAcceleration().getNorm() < PositionsConstants.Swerve.kMaxAcceleration.get();
        boolean atMinDist = ShootCalculator.getShootParams() == null || ShootCalculator.getShootParams().virtualDist() > PositionsConstants.Swerve.kHubMinDist.get();

        return switch (SwerveController.getInstance().getChannel()) {
            case "Look Hub" -> atAngleSmart && atAcceleration && atMinDist;

            case "Snap Ring" -> atPos && atAngle;

            case "Delivery" -> atAngle;

            case "Snap Angle" -> atAngle;

            case "Auto Trench" -> atPos && atAngle;

            default -> true;
        };
    }

    private SwerveSpeeds getDriveInput() {
        if (DriverStation.isAutonomous()) {
            if (autoInput == null)
                return new SwerveSpeeds(0, 0, 0, GeneralConstants.Swerve.kDriverFieldRelative);

            return autoInput.getAs(GeneralConstants.Swerve.kDriverFieldRelative);
        }

        Translation2d driverTranslation = new Translation2d(
            -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband),
            -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband)
        );
        driverTranslation = new Translation2d(Math.pow(driverTranslation.getNorm(), GeneralConstants.Swerve.kDriverPowFactor), driverTranslation.equals(new Translation2d()) ? Rotation2d.kZero : driverTranslation.getAngle())
            .times(GeneralConstants.Swerve.kDriverSpeedFactor)
            .times(SubsystemConstants.kSwerve.limits.maxSpeed);

        return new SwerveSpeeds(
            driverTranslation.getX(),
            driverTranslation.getY(),
            -Math.signum(driverRightX.getAsDouble()) * Math.pow(MathUtil.applyDeadband(Math.abs(driverRightX.getAsDouble()), GeneralConstants.Swerve.kJoystickDeadband), GeneralConstants.Swerve.kDriverRotationPowFactor) * GeneralConstants.Swerve.kDriverRotationSpeedFactor * SubsystemConstants.kSwerve.limits.maxAngularVelocity,
            GeneralConstants.Swerve.kDriverFieldRelative
        );
    }

    public void setAutoInput(ChassisSpeeds autoSpeeds) {
        autoInput = new SwerveSpeeds(autoSpeeds, false);
        SwerveController.getInstance().setControl(autoInput, "Auto");
    }

    public void slowForShoot() {
        Swerve.getInstance().setMaxSkidAcceleration(7);
        GeneralConstants.Swerve.kDriverSpeedFactor = 0.3;
    }

    public void unSlow() {
        Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
        GeneralConstants.Swerve.kDriverSpeedFactor = 1;
    }

    public Translation2d getAcceleration() {
        return accelerationCalculator.get();
    }

    public void stop() {
        if (!enabled)
            return;

        unSlow();

        if (DriverStation.isAutonomous()) {
            forceState(AUTO);
            SwerveController.getInstance().setChannel("Auto");
            SwerveController.getInstance().setControl(new SwerveSpeeds(), "Auto");
        }
        else {
            forceState(DRIVER);
            SwerveController.getInstance().setChannel("Driver");
            SwerveController.getInstance().setControl(new SwerveSpeeds(), "Driver");
        }
    }

    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    public boolean isReset() {
        if (!enabled)
            return true;

        return currentState == DRIVER || currentState == AUTO;
    }

    public void reset() {
        if (!enabled)
            return;

        stop();
        Swerve.getInstance().resetModulesToAbsolute();
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        SwerveController.getInstance().periodic();
        accelerationCalculator.calculate(Swerve.getInstance().getSpeeds().getAsFieldRelative().toTranslation());

        super.periodic();

        Logger.recordOutput("Robot/Swerve/Target", target);
        Logger.recordOutput("Robot/Swerve/Delivery Target", PositionsConstants.Swerve.getDeliveryTarget());
        Logger.recordOutput("Robot/Swerve/At Goal", atGoal());
    }
}