package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
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
    private DoubleSupplier leftX, leftY, rightX, rightY;
    private Command lookHubCommand;
    private Command lockCommand;
    private Command autoDriveCommand;
    private Command deliveryCommand;
    private Command[] commands;
    private Pose2d target = new Pose2d();

    public SwerveSubsystem(boolean enabled, boolean enabledMinimum, DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX, DoubleSupplier rightY) {
        this.enabled = enabled;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.rightY = rightY;

        if (enabled) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
            SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
            SwerveController.getInstance().setChannel("Driver");
        } else if (enabledMinimum) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
        }

        lookHubCommand = Commands.sequence(
            Commands.runOnce(() -> {
                for (Command command : commands) {
                    if (command != lookHubCommand)
                        command.cancel();
                }

                SwerveController.getInstance().setChannel("Look Hub");
            }),
            Commands.run(() -> {
                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(), RobotState.getInstance().getRobotPose().getY(), FieldConstants.getTranslationToHub().getAngle());
                SwerveController.getInstance().setControl(new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        SwerveController.getInstance().lookAt(FieldConstants.getHubPose().toPose2d(), Rotation2d.kZero),
                        GeneralConstants.Swerve.kDriverFieldRelative
                    ), "Look Hub");
            })
        );

        lockCommand = Commands.sequence(
            Commands.runOnce(() -> {
                for (Command command : commands) {
                    if (command != lockCommand)
                        command.cancel();
                }

                target = RobotState.getInstance().getRobotPose();

                SwerveController.getInstance().setChannel("Lock");
            }),
            Commands.run(() -> {
                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                SwerveController.getInstance().setControl(new SwerveInput(
                    pid.getX(),
                    pid.getY(),
                    SwerveController.getInstance().lookAt(FieldConstants.getHubPose().toPose2d(), Rotation2d.kZero),
                    true
                ), "Lock");
            })
        );

        autoDriveCommand = Commands.sequence(
            Commands.runOnce(() -> {
                for (Command command : commands) {
                    if (command != autoDriveCommand)
                        command.cancel();
                }

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

                target = new Pose2d(robot.getX() + transform.getX(),
                        robot.getY() + transform.getY(),
                        robot.getRotation().rotateBy(transform.getRotation()));

                SwerveController.getInstance().setChannel("Auto Drive");
            }),
            Commands.run(() -> {
                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                SwerveController.getInstance().setControl(new SwerveInput(
                        pid.getX(),
                        pid.getY(),
                        SwerveController.getInstance().lookAt(FieldConstants.getHubPose().toPose2d(), Rotation2d.kZero),
                        GeneralConstants.Swerve.kDriverFieldRelative
                ), "Auto Drive");
            })
        );

        deliveryCommand = Commands.sequence(
            Commands.runOnce(() -> {
                for (Command command : commands) {
                    if (command != deliveryCommand)
                        command.cancel();
                }

                target = RobotState.getInstance().getRobotPose();
                target = new Pose2d(target.getX(), target.getY(), Rotation2d.k180deg);

                SwerveController.getInstance().setChannel("Delivery");
            }),
            Commands.run(() -> {
                target = RobotState.getInstance().getRobotPose();
                target = new Pose2d(target.getX(), target.getY(), RobotState.getInstance().getTranslation(PositionsConstants.Swerve.getDeliveryTarget()).getAngle());

                SwerveController.getInstance().setControl(new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        SwerveController.getInstance().lookAt(PositionsConstants.Swerve.getDeliveryTarget(), Rotation2d.kZero),
                        true
                ), "Delivery");
            })
        );

        commands = new Command[] {
            lookHubCommand,
            lockCommand,
            autoDriveCommand,
            deliveryCommand
        };
    }

    @Override
    public boolean atGoal() {
        return RobotState.getInstance().getDistance(target) < PositionsConstants.Swerve.kHubPositionThreshold.get()
                && Math.abs(target.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) < PositionsConstants.Swerve.kHubAngleThreshold.get();
    }

    @Override
    public Pose2d getGoal() {
        return target;
    }

    public Command lookHub() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(lookHubCommand);
    }

    public Command lock() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(lockCommand);
    }

    public Command autoDrive() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(autoDriveCommand);
    }

    public Command deliveryDrive() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(deliveryCommand);
    }

    @Override
    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            for (Command command : commands) {
                if (command.isScheduled() && !command.isFinished())
                    command.cancel();
            }

            if (DriverStation.isAutonomous()){
                SwerveController.getInstance().setChannel("Auto");
                SwerveController.getInstance().setControl(new SwerveInput(), "Auto");
            }
            else{
                SwerveController.getInstance().setChannel("Driver");
                SwerveController.getInstance().setControl(new SwerveInput(), "Driver");
            }
        });
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        boolean commandsCanceled = true;
        for (Command command : commands) {
            if (command.isScheduled() && !command.isFinished()) {
                commandsCanceled = false;
                break;
            }
        }

        return commandsCanceled
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
            new SwerveInput(
                -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                -MathUtil.applyDeadband(rightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor,
                GeneralConstants.Swerve.kDriverFieldRelative
            )), "Driver");

        SwerveController.getInstance().periodic();

        Logger.recordOutput("Swerve/Look Hub Command", lookHubCommand.isScheduled() && !lookHubCommand.isFinished());
        Logger.recordOutput("Swerve/Lock Command", lockCommand.isScheduled() && !lockCommand.isFinished());
        Logger.recordOutput("Swerve/Auto Drive Command", autoDriveCommand.isScheduled() && !autoDriveCommand.isFinished());
        Logger.recordOutput("Swerve/Delivery Command", deliveryCommand.isScheduled() && !deliveryCommand.isFinished());

        Logger.recordOutput("Swerve/Target", target);
        Logger.recordOutput("Swerve/Delivery Target", PositionsConstants.Swerve.getDeliveryTarget());
    }
}