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
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {
    private boolean enabled;
    private Pose2d target;
    private Command reefCommand;

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
            SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
            SwerveController.getInstance().setChannel("Driver");

            target = Pose2d.kZero;

            reefCommand = Commands.sequence(
                    Commands.runOnce(() -> {
                        SwerveController.getInstance().setChannel("AutoReef");
                        SwerveController.getInstance().resetLookAt();

                        target = FieldConstants.nearestReef().pose.toPose2d();
                        target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));

                        if (!RobotState.isInverseReef()) {
                            if (RobotState.isRightReef()) {
                                target = target.transformBy(new Transform2d(-PositionsConstants.AutoDrive.kDistFromReef,
                                    PositionsConstants.AutoDrive.kRightOffsetInverse,
                                        Rotation2d.kZero));
                            } else {
                                target = target.transformBy(new Transform2d(-PositionsConstants.AutoDrive.kDistFromReef,
                                    PositionsConstants.AutoDrive.kLeftOffset,
                                        Rotation2d.kZero));
                            }
                        } else {
                            if (RobotState.isRightReef()) {
                                target = target.transformBy(new Transform2d(-PositionsConstants.AutoDrive.kDistFromReefInverse,
                                    PositionsConstants.AutoDrive.kRightOffsetInverse,
                                        Rotation2d.k180deg));
                            } else {
                                target = target.transformBy(new Transform2d(-PositionsConstants.AutoDrive.kDistFromReefInverse,
                                    PositionsConstants.AutoDrive.kLeftOffsetInverse,
                                        Rotation2d.k180deg));
                            }
                        }
                    }),
                    Commands.run(() -> {
                        Translation2d translation = RobotState.getInstance().getTranslation(target);
                        Translation2d dir = translation.div(translation.getNorm());
                        double velocity = f(translation.getNorm());

                        SwerveController.getInstance().setControl(
                                new SwerveInput(
                                        velocity * dir.getX(),
                                        velocity * dir.getY(),
                                        SwerveController.getInstance().lookAt(target.getRotation()),
                                        true
                                ), "AutoReef"
                        );
                    })
            );
        }
    }

    public void swerveDrive(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(rightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor,
                        GeneralConstants.Swerve.kDriverFieldRelative
                )), "Driver");
    }

    double f(double error){
        double a = 2;
        double b = 2;
        return Math.pow(a * error, 1 / b);
    }

    public Command autoDriveToReef() {
        return reefCommand;
    }

    public Command autoBackFromReef(double seconds) {
        return Commands.runOnce(() -> {
            reefCommand.cancel();
            SwerveController.getInstance().setChannel("BackReef");
            SwerveController.getInstance().setControl(new SwerveInput(RobotState.isInverseReef() ? 2 : -2, 0, 0, false), "BackReef");
        }).andThen(Commands.waitSeconds(seconds)).andThen(close());
    }

    public double distFromGoal() {
        return Math.abs(RobotState.getInstance().getDistance(target));
    }

    public boolean atGoal() {
        return distFromGoal() < PositionsConstants.AutoDrive.kPositionThreshold
                && Math.abs(target.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getRadians()) < PositionsConstants.AutoDrive.kRotationThreshold.getRadians();
    }

    public Command close() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            reefCommand.cancel();

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

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
                close(),
                Commands.runOnce(() -> Swerve.getInstance().resetModulesToAbsolute())
        );
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        SwerveController.getInstance().periodic();

        double accLimitAt0 = 65;
        double accLimitAt10 = 24;
        double elevatorHeight = RobotContainer.getElevator().getHeight();
        double accLimit = (accLimitAt0 - accLimitAt10) / -10 * elevatorHeight + accLimitAt0;
        SubsystemConstants.kSwerve.limits.maxSkidAcceleration = accLimit;


        Logger.recordOutput("Swerve/Reef Command", reefCommand.isScheduled() && !reefCommand.isFinished());
        Logger.recordOutput("Swerve/Reef Target", target);
        Logger.recordOutput("Swerve/Acceleration Limit", accLimit);
    }
}