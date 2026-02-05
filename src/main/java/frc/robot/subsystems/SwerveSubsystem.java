package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.RobotState;
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

    public Command lookHub() {
        if (!enabled)
            return Commands.none();

        return backgroundCommand.setNewTaskCommand(Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Look Hub");
                SwerveController.getInstance().resetLookAt();
            }),
            Commands.run(() -> {
//                double relativeYVel = Swerve.getInstance().getSpeeds().vyMetersPerSecond;
//                double relativeYWantedVel = Swerve.getInstance().getWantedSpeeds().vyMetersPerSecond;

//                double angleFix = (PositionsConstants.Swerve.getAngleFix(Math.abs(relativeYWantedVel)) * -Math.signum(relativeYWantedVel)) * 0.8
//                        + (PositionsConstants.Swerve.getAngleFix(Math.abs(relativeYVel)) * -Math.signum(relativeYVel)) * 0.2;

//                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(), RobotState.getInstance().getRobotPose().getY(), FieldConstants.getTranslationToHub().getAngle().rotateBy(Rotation2d.fromDegrees(angleFix)));

                Rotation2d angleHub = RobotState.getInstance().getAngleToHub();
                if (RobotState.getInstance().getDistToHub() < PositionsConstants.Swerve.kTargetMinThreshold.get())
                    angleHub = FieldConstants.getTranslationToHub().getAngle();
                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(), RobotState.getInstance().getRobotPose().getY(), angleHub);

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

//                Logger.recordOutput("Swerve/Angle Fix", angleFix);
            })
        ));
    }

    public Command lock() {
        if (!enabled)
            return Commands.none();

        return backgroundCommand.setNewTaskCommand(Commands.sequence(
            Commands.runOnce(() -> {
                target = RobotState.getInstance().getRobotPose();

                SwerveController.getInstance().setChannel("Lock");
            }),
//            Commands.run(() -> {
//                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
//                SwerveController.getInstance().setControl(new SwerveInput(
//                    pid.getX(),
//                    pid.getY(),
//                    SwerveController.getInstance().lookAt(target.getRotation()),
//                    true
//                ), "Lock");
//            })
            Swerve.getInstance().lockWheelsToX()
        ));
    }

    public Command autoDrive() {
        if (!enabled)
            return Commands.none();

        return backgroundCommand.setNewTaskCommand(Commands.sequence(
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

    public Command deliveryDrive() {
        if (!enabled)
            return Commands.none();

        return backgroundCommand.setNewTaskCommand(Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Delivery");
            }),
            Commands.run(() -> {
                target = RobotState.getInstance().getRobotPose();
                target = new Pose2d(target.getX(), target.getY(), RobotState.getInstance().getTranslation(PositionsConstants.Swerve.getDeliveryTarget()).getAngle());

                SwerveController.getInstance().setControl(new SwerveSpeeds(
                    -MathUtil.applyDeadband(driverLeftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    -MathUtil.applyDeadband(driverLeftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    true
                ), "Delivery");
            })
        ));
    }

    @Override
    public boolean atGoal() {
        return RobotState.getInstance().getDistance(target) < PositionsConstants.Swerve.kPositionThreshold.get()
            && Math.abs(target.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) < (PositionsConstants.Swerve.kAngleBaseThreshold.get() + PositionsConstants.Swerve.kAngleCoefficient.get() * FieldConstants.getDistToHub())
            && Math.abs(Swerve.getInstance().getWantedSpeeds().getSpeed() - Swerve.getInstance().getSpeeds().getSpeed()) < PositionsConstants.Swerve.kSpeedDifferenceThreshold.get()
            && RobotState.getInstance().getDistToHub() > PositionsConstants.Swerve.kHubMinDist.get();
    }

    @Override
    public Pose2d getGoal() {
        return target;
    }

    public void setAutoInput(ChassisSpeeds autoSpeeds) {
        autoInput = new SwerveSpeeds(autoSpeeds, false);
        SwerveController.getInstance().setControl(autoInput, "Auto");
    }

    public Command slowForShoot() {
        return Commands.runOnce(() -> {
            SubsystemConstants.kSwerve.limits.maxSkidAcceleration = 12.5;
            GeneralConstants.Swerve.kDriverSpeedFactor = 0.3;
        });
    }

    @Override
    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            backgroundCommand.stop();

            SubsystemConstants.kSwerve.limits.maxSkidAcceleration = 80;
            GeneralConstants.Swerve.kDriverSpeedFactor = 1;

            if (DriverStation.isAutonomous()){
                SwerveController.getInstance().setChannel("Auto");
                SwerveController.getInstance().setControl(new SwerveSpeeds(), "Auto");
            }
            else{
                SwerveController.getInstance().setChannel("Driver");
                SwerveController.getInstance().setControl(new SwerveSpeeds(), "Driver");
            }
        });
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
            stop(),
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