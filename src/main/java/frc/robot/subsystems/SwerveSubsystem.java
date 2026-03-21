package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
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
    private boolean stoppedAutoTrench = false;

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
        addOmniEdge(DRIVER, () -> Commands.runOnce(() -> {
            SwerveController.getInstance().setChannel("Driver");
        }));
        addStateCommand(DRIVER, Commands.run(() -> {
            SwerveController.getInstance().setControl(getDriveInput(), "Driver");
        }));


        addOmniEdge(AUTO, () -> Commands.runOnce(() -> {
            SwerveController.getInstance().setChannel("Auto");
        }));
        addStateCommand(AUTO, Commands.run(() -> {
            SwerveController.getInstance().setControl(getDriveInput(), "Auto");
        }));


        addOmniEdge(LOOK_HUB, () -> Commands.runOnce(() -> {
            target = new Pose2d(RobotState.get().getTranslation(), ShootCalculator.getShootParams().angle());
            lastLookHubTargetAngle = target.getRotation();
            slowForShoot();
            SwerveController.getInstance().setChannel("Look Hub");
        }));
        addStateCommand(LOOK_HUB, Commands.run(() -> {
            target = new Pose2d(RobotState.get().getTranslation(), ShootCalculator.getShootParams().angle());

            double FF = target.getRotation().minus(lastLookHubTargetAngle).div(0.02).times(PositionsConstants.Swerve.Hub.lookHubFF.get()).getRadians();
            Logger.recordOutput("SwerveSubsystem/Virtual Target Rotational Velocity", target.getRotation().minus(lastLookHubTargetAngle).div(0.02).getRadians());
            Logger.recordOutput("SwerveSubsystem/Look Hub FF", FF);
            lastLookHubTargetAngle = target.getRotation();

            SwerveController.getInstance().setControl(new SwerveSpeeds(
                getDriveInput().vxMetersPerSecond,
                getDriveInput().vyMetersPerSecond,
                SwerveController.getInstance().lookAt(target.getRotation()) + FF,
                GeneralConstants.Swerve.kDriverFieldRelative
            ), "Look Hub");
        }).finallyDo(this::unSlow));


        addOmniEdge(DELIVERY, () -> Commands.runOnce(() -> {
            target = new Pose2d(RobotState.get().getRobotPose().getX(), RobotState.get().getRobotPose().getY(), ShootCalculator.getShootParams().angle());
            SwerveController.getInstance().setChannel("Delivery");
        }));
        addStateCommand(DELIVERY, Commands.run(() -> {
            target = new Pose2d(RobotState.get().getRobotPose().getX(), RobotState.get().getRobotPose().getY(), ShootCalculator.getShootParams().angle());

            SwerveController.getInstance().setControl(new SwerveSpeeds(
                getDriveInput().vxMetersPerSecond,
                getDriveInput().vyMetersPerSecond,
                SwerveController.getInstance().lookAt(target.getRotation()),
                GeneralConstants.Swerve.kDriverFieldRelative
            ), "Delivery");
        }));


        addOmniEdge(SNAP_ANGLE, () -> Commands.runOnce(() -> {
            target = new Pose2d(
                RobotState.get().getRobotPose().getX(),
                RobotState.get().getRobotPose().getY(),
                Rotation2d.fromDegrees(Math.round(RobotState.get().getRotation().getDegrees() / 90.0) * 90.0)
            );
            SwerveController.getInstance().setChannel("Snap Angle");
        }));
        addStateCommand(SNAP_ANGLE, Commands.run(() -> {
            target = new Pose2d(
                RobotState.get().getRobotPose().getX(),
                RobotState.get().getRobotPose().getY(),
                Rotation2d.fromDegrees(Math.round(target.getRotation().getDegrees() / 90.0) * 90.0)
            );

            SwerveController.getInstance().setControl(new SwerveSpeeds(
                getDriveInput().vxMetersPerSecond,
                getDriveInput().vyMetersPerSecond,
                SwerveController.getInstance().lookAt(target.getRotation()),
                GeneralConstants.Swerve.kDriverFieldRelative
            ), "Snap Angle");
        }));


        addOmniEdge(AUTO_TRENCH, () -> Commands.runOnce(() -> {
            Pose2d robotPose = ShootCalculator.predict(
                RobotState.get().getRobotPose(),
                Swerve.getInstance().getSpeeds().getAsFieldRelative(),
                new SwerveSpeeds(),
                PositionsConstants.Swerve.AutoTrench.kPredict.get()
            ).pose();

            boolean isAtRightTrench = FieldConstants.atRightTrench(robotPose);

            target = isAtRightTrench ? FieldConstants.getRightTrenchPose() : FieldConstants.getLeftTrenchPose();
            target = new Pose2d(
                RobotState.get().getRobotPose().getX(),
                target.getY(),
                Rotation2d.fromDegrees(Math.round(RobotState.get().getRobotPose().getRotation().getDegrees() / 90) * 90)
            );

            SwerveController.getInstance().setChannel("Auto Trench");
        }));
        addStateCommand(AUTO_TRENCH, Commands.run(() -> {
            Pose2d robotPose = ShootCalculator.predict(
                RobotState.get().getRobotPose(),
                Swerve.getInstance().getSpeeds().getAsFieldRelative(),
                new SwerveSpeeds(),
                PositionsConstants.Swerve.AutoTrench.kPredict.get()
            ).pose();

            Logger.recordOutput("SwerveSubsystem/Trench Robot Predict", robotPose);
            Logger.recordOutput("SwerveSubsystem/Trench Threshold", new Pose3d(FieldConstants.getLeftTrenchPose().getX() - PositionsConstants.Swerve.AutoTrench.kThreshold.get(), FieldConstants.getLeftTrenchPose().getY(), 0.5, new Rotation3d(0, -Math.PI / 2, 0)));
            Logger.recordOutput("SwerveSubsystem/Trench Y Threshold", new Pose3d(FieldConstants.getLeftTrenchPose().getX() - PositionsConstants.Swerve.AutoTrench.kThreshold.get(), FieldConstants.getLeftTrenchPose().getY() - PositionsConstants.Swerve.AutoTrench.kYThreshold.get(), 0.5, new Rotation3d(0, -Math.PI / 2, 0)));

            boolean isAtRightTrench = FieldConstants.atRightTrench(robotPose);

            if (!FieldConstants.atLeftTrench(robotPose) && !isAtRightTrench) {
                SwerveController.getInstance().setControl(getDriveInput(), "Auto Trench");
                return;
            }

            target = isAtRightTrench ? FieldConstants.getRightTrenchPose() : FieldConstants.getLeftTrenchPose();

            double dist = robotPose.getTranslation().getDistance(target.getTranslation());
            double strength = Math.pow(Math.pow(PositionsConstants.Swerve.AutoTrench.kMaxStrength.get(), 1 / PositionsConstants.Swerve.AutoTrench.kExponent.get()) - Math.pow(PositionsConstants.Swerve.AutoTrench.kMaxStrength.get(), 1 / PositionsConstants.Swerve.AutoTrench.kExponent.get()) * (dist / PositionsConstants.Swerve.AutoTrench.kThreshold.get()), PositionsConstants.Swerve.AutoTrench.kExponent.get());
            strength = MathUtil.clamp(strength, 0, 1);
            Logger.recordOutput("SwerveSubsystem/Trench Strength", strength);

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

            SwerveController.getInstance().setControl(new SwerveSpeeds(
                getDriveInput().vxMetersPerSecond,
                getDriveInput().vyMetersPerSecond * (1 - strength) + pid.getY() * strength,
                getDriveInput().omegaRadiansPerSecond * (1 - strength) + SwerveController.getInstance().lookAt(target.getRotation()) * strength,
                getDriveInput().fieldRelative
            ), "Auto Trench");
        }));


        addStateEnd(SNAP_ANGLE, this::atGoal, DRIVER);

//        addStateEnd(DRIVER,
//            () -> (FieldConstants.atRightTrench() || FieldConstants.atLeftTrench()) && !stoppedAutoTrench,
//        AUTO_TRENCH);
//
//        addStateEnd(AUTO_TRENCH,
//            () -> !FieldConstants.atRightTrench() && !FieldConstants.atLeftTrench(),
//            DRIVER);
    }

    public boolean atGoal() {
        boolean atPos = RobotState.get().getDistance(target) < PositionsConstants.Swerve.kPositionThreshold.get();
        boolean atAngleSmart = Math.abs(target.getRotation().minus(RobotState.get().getRobotPose().getRotation()).getDegrees()) < (PositionsConstants.Swerve.kAngleThresholdBase.get() + PositionsConstants.Swerve.kAngleThresholdCoefficient.get() * FieldConstants.getDistToHub().get());
        boolean atAngle = Math.abs(target.getRotation().minus(RobotState.get().getRobotPose().getRotation()).getDegrees()) < PositionsConstants.Swerve.kAngleThreshold.get();
        boolean atAcceleration = getAcceleration().getNorm() < PositionsConstants.Swerve.kMaxAcceleration.get();
        boolean atMinDist = ShootCalculator.getShootParams() == null || ShootCalculator.getShootParams().virtualDist() > PositionsConstants.Swerve.Hub.kHubMinDist.get();

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

        stoppedAutoTrench = true;

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

        if (stoppedAutoTrench && !FieldConstants.atLeftTrench() && !FieldConstants.atRightTrench())
            stoppedAutoTrench = false;

        super.periodic();

        Logger.recordOutput("SwerveSubsystem/Target", target);
        Logger.recordOutput("SwerveSubsystem/Delivery Target", PositionsConstants.Swerve.Delivery.getDeliveryTarget());
        Logger.recordOutput("SwerveSubsystem/At Goal", atGoal());
    }
}