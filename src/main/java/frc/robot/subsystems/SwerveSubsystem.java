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
import frc.lib.NinjasLib.commands.BackgroundCommand;
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
    private BackgroundCommand backgroundCommand;
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
                Translation2d robotRelativeVelocity = new Translation2d(Swerve.getInstance().getChassisSpeeds(false).vxMetersPerSecond, Swerve.getInstance().getChassisSpeeds(false).vyMetersPerSecond);
                double angleFix = PositionsConstants.Swerve.getAngleFix(Math.abs(robotRelativeVelocity.getY())) * -Math.signum(robotRelativeVelocity.getY());
                target = new Pose2d(RobotState.getInstance().getRobotPose().getX(), RobotState.getInstance().getRobotPose().getY(), FieldConstants.getTranslationToHub().getAngle().rotateBy(Rotation2d.fromDegrees(angleFix)));

                SwerveController.getInstance().setControl(new SwerveInput(
                    -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
//                    -MathUtil.applyDeadband(rightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor * SubsystemConstants.kSwerve.limits.maxAngularVelocity,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    GeneralConstants.Swerve.kDriverFieldRelative
                ), "Look Hub");

                Logger.recordOutput("Swerve/Angle Fix", angleFix);
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
            Commands.run(() -> {
                Translation2d pid = SwerveController.getInstance().pidTo(target.getTranslation());
                SwerveController.getInstance().setControl(new SwerveInput(
                    pid.getX(),
                    pid.getY(),
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    true
                ), "Lock");
            })
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
                SwerveController.getInstance().setControl(new SwerveInput(
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

                SwerveController.getInstance().setControl(new SwerveInput(
                    -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                    SwerveController.getInstance().lookAt(target.getRotation()),
                    true
                ), "Delivery");
            })
        ));
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

    @Override
    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            backgroundCommand.stop();

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
            new SwerveInput(
                -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                -MathUtil.applyDeadband(rightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor,
                GeneralConstants.Swerve.kDriverFieldRelative
            )), "Driver");

        SwerveController.getInstance().periodic();

        Logger.recordOutput("Swerve/Auto Command", backgroundCommand.isRunning());
        Logger.recordOutput("Swerve/Target", target);
        Logger.recordOutput("Swerve/Delivery Target", PositionsConstants.Swerve.getDeliveryTarget());
    }
}