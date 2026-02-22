package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.States;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase implements
    ISubsystem.Resettable,
    ISubsystem.GoalOriented<Pose2d>,
    ISubsystem.Stoppable
{
    private boolean enabled;
    private DoubleSupplier driverLeftX, driverLeftY, driverRightX, driverRightY;
    private SwerveSpeeds autoInput;
    private BackgroundCommand backgroundCommand;
    private Pose2d target = new Pose2d();

    public SwerveSubsystem(boolean enabled, boolean enabledMinimum, DoubleSupplier driverLeftX, DoubleSupplier driverLeftY, DoubleSupplier driverRightX, DoubleSupplier driverRightY) {
        this.enabled = enabled;
        this.driverLeftX = driverLeftX;
        this.driverLeftY = driverLeftY;
        this.driverRightX = driverRightX;
        this.driverRightY = driverRightY;

        if (enabled) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
            SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
            SwerveController.getInstance().setChannel("Driver");

            backgroundCommand = new BackgroundCommand();
        } else if (enabledMinimum) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        }
    }

    public void lookHub() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Look Hub");
                SwerveController.getInstance().resetLookAt();
            }),
            Commands.run(() -> {
                Rotation2d angleToHub = RobotState.getInstance().getLookaheadTargetAngle(FieldConstants.getHubPose());
                if (RobotState.getInstance().getLookaheadTargetDist(FieldConstants.getHubPose()) < PositionsConstants.Swerve.kTargetMinThreshold.get())
                    angleToHub = FieldConstants.getTranslationToHub().getAngle();
                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(), RobotState.getInstance().getRobotPose().getY(), angleToHub);

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Look Hub");
            })
        ));
    }

    public void lock() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.runOnce(() -> {
            target = RobotState.getInstance().getRobotPose();

            SwerveController.getInstance().setChannel("Lock");
            Swerve.getInstance().lockWheelsToX();
        }));
    }

    public void snapRing() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> {
                double dist = FieldConstants.getDistToHub();
                Pose2d robot = RobotState.getInstance().getRobotPose();
                Transform2d transform = new Transform2d();

                if (dist > PositionsConstants.Swerve.kHubMaxDist.get()) {
                    Translation2d dir = FieldConstants.getTranslationToHub().div(dist);
                    transform = new Transform2d(dir.times(dist - PositionsConstants.Swerve.kHubMaxDist.get()), Rotation2d.kZero);
                } else if (dist < PositionsConstants.Swerve.kHubMinDist.get()) {
                    Translation2d dir = FieldConstants.getTranslationToHub().div(dist).unaryMinus();
                    transform = new Transform2d(dir.times(PositionsConstants.Swerve.kHubMinDist.get() - dist), Rotation2d.kZero);
                }

                target = new Pose2d(
                    robot.getX() + transform.getX(),
                    robot.getY() + transform.getY(),
                    FieldConstants.getTranslationToHub().getAngle());

                SwerveController.getInstance().setChannel("Auto Drive");
            }),
            Commands.run(() -> {
                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    pid.getX(),
                    pid.getY(),
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Auto Drive");
            })
        ));
    }

    public void delivery() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Delivery");
            }),
            Commands.run(() -> {
                Pose3d deliveryTarget = new Pose3d(PositionsConstants.Swerve.getDeliveryTarget());
                deliveryTarget = new Pose3d(deliveryTarget.getX(), deliveryTarget.getY(), 0, Rotation3d.kZero);

                Rotation2d angleToDelivery = RobotState.getInstance().getLookaheadTargetAngle(deliveryTarget);
                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(), RobotState.getInstance().getRobotPose().getY(), angleToDelivery);

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Delivery");
            })
        ));
    }

    public void snapAngle() {
        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> {
                target = RobotState.getInstance().getRobotPose();
                target = new Pose2d(target.getX(), target.getY(), Rotation2d.fromDegrees(Math.round(target.getRotation().getDegrees() / 90.0) * 90.0));
                SwerveController.getInstance().setChannel("Snap Angle");
            }),
            Commands.run(() -> {
                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(),
                    RobotState.getInstance().getRobotPose().getY(),
                    Rotation2d.fromDegrees(Math.round(target.getRotation().getDegrees() / 90.0) * 90.0));

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    getDriveInput().vxMetersPerSecond,
                    getDriveInput().vyMetersPerSecond,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Snap Angle");
            })
        ));
    }

    private static final double autoTrenchMaxStrength = 1;
    private static final double autoTrenchExp = 0.75;
    public void autoTrench() {
        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Auto Trench");
            }),
            Commands.run(() -> {
                if (!nearLeftTrench() && !nearRightTrench())
                    return;

                boolean isRightTrench = nearRightTrench();

                target = isRightTrench ? FieldConstants.getRightTrenchPose() : FieldConstants.getLeftTrenchPose();
                target = new Pose2d(
                    RobotState.getInstance().getRobotPose().getX(),
                    target.getY(),
                    Rotation2d.fromDegrees(Math.round(RobotState.getInstance().getRobotPose().getRotation().getDegrees() / 90) * 90)
                );

                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                if (!getDriveInput().fieldRelative) {
                    ChassisSpeeds pidRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(pid.getX(), pid.getY(), 0), RobotState.getInstance().getRobotPose().getRotation());
                    pid = new Translation2d(pidRobotRelative.vxMetersPerSecond, pidRobotRelative.vyMetersPerSecond);
                }

                double dist = nearRightTrench()
                    ? RobotState.getInstance().getDistance(FieldConstants.getRightTrenchPose())
                    : RobotState.getInstance().getDistance(FieldConstants.getLeftTrenchPose());
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
    }

    private SwerveSpeeds getDriveInput() {
        if (DriverStation.isAutonomous()) {
            if (autoInput == null)
                return new SwerveSpeeds(0, 0, 0, GeneralConstants.Swerve.kDriverFieldRelative);

            return autoInput.getAs(GeneralConstants.Swerve.kDriverFieldRelative);
        }

        return new SwerveSpeeds(
            -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
            -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
            -MathUtil.applyDeadband(driverRightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor * SubsystemConstants.kSwerve.limits.maxAngularVelocity,
            GeneralConstants.Swerve.kDriverFieldRelative
        );
    }

    @Override
    public boolean atGoal() {
        boolean atPos = RobotState.getInstance().getDistance(target) < PositionsConstants.Swerve.kPositionThreshold.get();
        boolean atAngleSmart = Math.abs(target.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) < (PositionsConstants.Swerve.kAngleThresholdBase.get() + PositionsConstants.Swerve.kAngleThresholdCoefficient.get() * FieldConstants.getDistToHub());
        boolean atAngle = Math.abs(target.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) < PositionsConstants.Swerve.kAngleThreshold.get();
        boolean atAcceleration = RobotContainer.getRobotAcceleration().getNorm() < PositionsConstants.Swerve.kMaxAcceleration.get();
        boolean atMinDist = RobotState.getInstance().getLookaheadTargetDist(FieldConstants.getHubPose()) > PositionsConstants.Swerve.kHubMinDist.get();

        return (RobotState.getShootingMode() == States.ShootingMode.SNAP_RING ? atPos : true)
            && (RobotState.getShootingMode() == States.ShootingMode.DELIVERY ? atAngle : atAngleSmart)
            && (RobotState.getShootingMode() == States.ShootingMode.ON_MOVE ? atAcceleration : true)
            && (RobotState.getShootingMode() == States.ShootingMode.ON_MOVE ? atMinDist : true);
    }

    public boolean nearLeftTrench() {
        double leftTrenchDist = RobotState.getInstance().getDistance(FieldConstants.getLeftTrenchPose());
        double robotY = RobotState.getInstance().getRobotPose().getY();
        return leftTrenchDist < PositionsConstants.Swerve.kAutoTrenchThreshold.get() && Math.abs(FieldConstants.getLeftTrenchPose().getY() - robotY) < PositionsConstants.Swerve.kAutoTrenchYThreshold.get();
    }

    public boolean nearRightTrench() {
        double rightTrenchDist = RobotState.getInstance().getDistance(FieldConstants.getRightTrenchPose());
        double robotY = RobotState.getInstance().getRobotPose().getY();
        return rightTrenchDist < PositionsConstants.Swerve.kAutoTrenchThreshold.get() && Math.abs(FieldConstants.getRightTrenchPose().getY() - robotY) < PositionsConstants.Swerve.kAutoTrenchYThreshold.get();
    }

    public boolean atAngle() {
        return Math.abs(target.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) < PositionsConstants.Swerve.kAngleThreshold.get();
    }

    @Override
    public Pose2d getGoal() {
        return target;
    }

    public void setAutoInput(ChassisSpeeds autoSpeeds) {
        autoInput = new SwerveSpeeds(autoSpeeds, false);
        SwerveController.getInstance().setControl(autoInput, "Auto");
    }

    public void slowForShoot() {
        Swerve.getInstance().setMaxSkidAcceleration(15);
        GeneralConstants.Swerve.kDriverSpeedFactor = 0.3;
    }

    public void unSlow() {
        Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
        GeneralConstants.Swerve.kDriverSpeedFactor = 1;
    }

    @Override
    public void stop() {
        if (!enabled)
            return;

        backgroundCommand.stop();
        unSlow();

        if (DriverStation.isAutonomous()){
            SwerveController.getInstance().setChannel("Auto");
            SwerveController.getInstance().setControl(new SwerveSpeeds(), "Auto");
        }
        else{
            SwerveController.getInstance().setChannel("Driver");
            SwerveController.getInstance().setControl(new SwerveSpeeds(), "Driver");
        }
    }

    @Override
    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return !backgroundCommand.isRunning()
            && (SwerveController.getInstance().getChannel().equals("Driver") || SwerveController.getInstance().getChannel().equals("Auto"));
    }

    @Override
    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
            stopCmd(),
            Commands.runOnce(() -> Swerve.getInstance().resetModulesToAbsolute())
        );
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
            new SwerveSpeeds(
                -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                -MathUtil.applyDeadband(driverRightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor,
                GeneralConstants.Swerve.kDriverFieldRelative
            )), "Driver");

        SwerveController.getInstance().periodic();

        Logger.recordOutput("Robot/Swerve/Auto Command", backgroundCommand.isRunning());
        Logger.recordOutput("Robot/Swerve/Target", target);
        Logger.recordOutput("Robot/Swerve/Delivery Target", PositionsConstants.Swerve.getDeliveryTarget());
        Logger.recordOutput("Robot/Swerve/At Goal", atGoal());
    }
}