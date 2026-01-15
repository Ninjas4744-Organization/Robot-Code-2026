package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase implements
    ISubsystem.Resettable,
    ISubsystem.GoalOriented<Rotation2d>,
    ISubsystem.Stoppable
{
    private boolean enabled;
    private DoubleSupplier leftX, leftY, rightX, rightY;
    private Command lookHubCommand;
    private Command lockCommand;
    private Command autoDriveCommand;
    private Command[] commands;

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

                }

                SwerveController.getInstance().setChannel("Look Hub");
            }),
            Commands.run(() -> {
                SwerveController.getInstance().setControl(new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        SwerveController.getInstance().lookAt(FieldConstants.getHubPose().toPose2d(), Rotation2d.kZero),
                        GeneralConstants.Swerve.kDriverFieldRelative
                    ), "Shoot Prepare");
            })
        );

        lockCommand = Commands.sequence(
            Commands.runOnce(() -> {
                if (lookHubCommand.isScheduled() && !lookHubCommand.isFinished())
                    lookHubCommand.cancel();

                SwerveController.getInstance().setChannel("Lock");
            }),
            Commands.run(() -> {
                SwerveController.getInstance().setControl(new SwerveInput(
                    0,
                    0,
                    SwerveController.getInstance().lookAt(FieldConstants.getHubPose().toPose2d(), Rotation2d.kZero),
                    true
                ), "Lock");
            })
        );

        autoDriveCommand = Commands.sequence(
            Commands.runOnce(() -> {
                if (lockCommand.isScheduled() && !lockCommand.isFinished())
                    lockCommand.cancel();

                SwerveController.getInstance().setChannel("Shoot Prepare");
            }),
            Commands.run(() -> {
                SwerveController.getInstance().setControl(new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor * SubsystemConstants.kSwerve.limits.maxSpeed,
                        SwerveController.getInstance().lookAt(FieldConstants.getHubPose().toPose2d(), Rotation2d.kZero),
                        GeneralConstants.Swerve.kDriverFieldRelative
                ), "Shoot Prepare");
            })
        );

        commands = new Command[] {
            lookHubCommand,
            lockCommand,
            autoDriveCommand
        };
    }

    public Command lookHub() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(lookHubCommand);
    }

    @Override
    public boolean atGoal() {
        return Math.abs(FieldConstants.getTranslationToHub().getAngle().minus(RobotState.getInstance().getRobotPose().getRotation()).getRadians()) < GeneralConstants.Swerve.kHubAngleThreshold.getRadians();
    }

    @Override
    public Rotation2d getGoal() {
        return FieldConstants.getTranslationToHub().getAngle();
    }

    public Command lock() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(lockCommand);
    }

    @Override
    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            if (lookHubCommand.isScheduled() && !lookHubCommand.isFinished())
                lookHubCommand.cancel();
            if (lockCommand.isScheduled() && !lockCommand.isFinished())
                lockCommand.cancel();

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

        return !lookHubCommand.isScheduled()
            && !lockCommand.isScheduled()
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
    }
}