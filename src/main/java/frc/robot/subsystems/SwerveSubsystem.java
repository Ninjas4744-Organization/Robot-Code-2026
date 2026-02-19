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

                double vx = 0, vy = 0;
                if (DriverStation.isAutonomous()) {
                    if (autoInput != null) {
                        if (GeneralConstants.Swerve.kDriverFieldRelative) {
                            vx = autoInput.getAsFieldRelative(RobotState.getInstance().getRobotPose().getRotation()).vxMetersPerSecond;
                            vy = autoInput.getAsFieldRelative(RobotState.getInstance().getRobotPose().getRotation()).vyMetersPerSecond;
                        } else {
                            vx = autoInput.getAsRobotRelative(RobotState.getInstance().getRobotPose().getRotation()).vxMetersPerSecond;
                            vy = autoInput.getAsRobotRelative(RobotState.getInstance().getRobotPose().getRotation()).vyMetersPerSecond;
                        }
                    }
                } else {
                    vx = -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed;
                    vy = -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed;
                }

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    vx,
                    vy,
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
                    -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
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
                    -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Snap Angle");
            })
        ));
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

        Logger.recordOutput("Swerve/Auto Command", backgroundCommand.isRunning());
        Logger.recordOutput("Swerve/Target", target);
        Logger.recordOutput("Swerve/Delivery Target", PositionsConstants.Swerve.getDeliveryTarget());
        Logger.recordOutput("Swerve/At Goal", atGoal());
    }
}