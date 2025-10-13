package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.StateMachine;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveSubsystem extends SubsystemBase {
    private boolean enabled;
    private Pose2d finalTarget;
    private Pose2d target;
    private Command driveToReefCommand;
    private Command driveToCoralCommand;

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(Constants.Swerve.kSwerveConstants));
            SwerveController.setInstance(new SwerveController(Constants.Swerve.kSwerveControllerConstants));
            SwerveController.getInstance().setChannel("Driver");

            finalTarget = Pose2d.kZero;
            target = Pose2d.kZero;
            autoReefAnglePID.enableContinuousInput(-Math.PI, Math.PI);

            SmartDashboard.putNumber("Dist Offset", 0);
            SmartDashboard.putNumber("Dist Inverse Offset", 0);
            SmartDashboard.putNumber("Dist L4 Offset", 0);
            SmartDashboard.putNumber("Right Offset", 0);
            SmartDashboard.putNumber("Left Offset", 0);
            SmartDashboard.putNumber("Right Inverse Offset", 0);
            SmartDashboard.putNumber("Left Inverse Offset", 0);
        }
    }

    public void swerveDrive(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), Constants.Swerve.kJoystickDeadband) * Constants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), Constants.Swerve.kJoystickDeadband) * Constants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(rightX.getAsDouble(), Constants.Swerve.kJoystickDeadband) * Constants.Swerve.kDriverRotationSpeedFactor,
                        Constants.Swerve.kDriverFieldRelative
                )), "Driver");
    }

    private ProfiledPIDController autoReefAnglePID = new ProfiledPIDController(6, 0, 0.1, new TrapezoidProfile.Constraints(6, 12));
    double f(double error){
        double a = 2;
        double b = 2;
        return Math.pow(a * error, 1 / b);
    }

    private boolean isAutoDriveReefStage1 = true;
    public Supplier<Command> autoDriveToReef(BooleanSupplier isRightSide) {
        return () -> {
            driveToReefCommand = Commands.sequence(
                    Commands.runOnce(() -> {
                        Swerve.getInstance().resetModulesToAbsolute();
                        SwerveController.getInstance().setChannel("AutoReef");

                        finalTarget = Constants.Field.nearestReef().pose.toPose2d();
                        finalTarget = new Pose2d(finalTarget.getTranslation(), finalTarget.getRotation().rotateBy(Rotation2d.k180deg));

                        double distOffset         = Math.round(SmartDashboard.getNumber("Dist Offset", 0) * 100) / 100.0;
                        double distInverseOffset  = Math.round(SmartDashboard.getNumber("Dist Inverse Offset", 0) * 100) / 100.0;
                        double distL4Offset       = Math.round(SmartDashboard.getNumber("Dist L4 Offset", 0) * 100) / 100.0;
                        double rightOffset        = Math.round(SmartDashboard.getNumber("Right Offset", 0) * 100) / 100.0;
                        double leftOffset         = Math.round(SmartDashboard.getNumber("Left Offset", 0) * 100) / 100.0;
                        double rightInverseOffset = Math.round(SmartDashboard.getNumber("Right Inverse Offset", 0) * 100) / 100.0;
                        double leftInverseOffset  = Math.round(SmartDashboard.getNumber("Left Inverse Offset", 0) * 100) / 100.0;

                        boolean invertTarget = invertRobotOuttake();
                        finalTarget = finalTarget.transformBy(new Transform2d(RobotState.getL() == 4 ? (-Constants.AutoDrive.kDistFromReefL4 - distL4Offset) : (invertTarget ? (-Constants.AutoDrive.kDistFromReefInverse - distInverseOffset) : (-Constants.AutoDrive.kDistFromReef - distOffset)),
                                isRightSide.getAsBoolean() ? ((invertTarget ? (-Constants.AutoDrive.kRightSideInverseOffset - rightInverseOffset + (RobotState.getL() == 2 ? -Constants.AutoDrive.kRightSideInverseL2ExtraOffset : 0)) : (-Constants.AutoDrive.kRightSideOffset - rightOffset + (RobotState.getL() == 2 ? -Constants.AutoDrive.kRightSideL2ExtraOffset : 0))) - Constants.AutoDrive.kReefRodOffset) : ((invertTarget ? (Constants.AutoDrive.kLeftSideInverseOffset + leftInverseOffset + (RobotState.getL() == 2 ? Constants.AutoDrive.kLeftSideInverseL2ExtraOffset : 0)) : (Constants.AutoDrive.kLeftSideOffset + leftOffset + (RobotState.getL() == 2 ? Constants.AutoDrive.kLeftSideL2ExtraOffset : 0))) + Constants.AutoDrive.kReefRodOffset),
                                Rotation2d.kZero));
                        target = finalTarget.transformBy(new Transform2d(-Constants.AutoDrive.kDistBackFirstTarget, 0, Rotation2d.kZero));

                        if (invertTarget) {
                            finalTarget = new Pose2d(finalTarget.getTranslation(), finalTarget.getRotation().rotateBy(Rotation2d.k180deg));
                            target = new Pose2d(target.getTranslation(), target.getRotation().rotateBy(Rotation2d.k180deg));
                        }
                        StateMachine.getInstance().setOuttakeInversion(invertTarget);

                        autoReefAnglePID.reset(RobotState.getInstance().getRobotPose().getRotation().getRadians());
                    }),
                    Commands.run(() -> {
                        if (atFirstTarget(target) && isAutoDriveReefStage1) {
                            target = finalTarget;
                            isAutoDriveReefStage1 = false;
                        }

                        Translation2d translation = RobotState.getInstance().getTranslation(target);
                        Translation2d dir = translation.div(translation.getNorm());
                        double velocity = f(translation.getNorm());
                        double anglePID = autoReefAnglePID.calculate(RobotState.getInstance().getRobotPose().getRotation().getRadians(), target.getRotation().getRadians());

                        SwerveController.getInstance().setControl(
                                new SwerveInput(
                                        velocity * dir.getX(),
                                        velocity * dir.getY(),
                                        anglePID,
                                        true
                                ), "AutoReef"
                        );
                    })/*.until(this::atGoal)*/.andThen(Commands.runOnce(() -> SwerveController.getInstance().setControl(new SwerveInput(0, 0, 0, false), "AutoReef")))
            );
            return driveToReefCommand;
        };
    }

    public double distFromGoal() {
        return Math.abs(RobotState.getInstance().getDistance(finalTarget));
    }

    private boolean atFirstTarget(Pose2d target) {
        return Math.abs(RobotState.getInstance().getDistance(target)) < Constants.AutoDrive.kFirstDistThreshold;
    }

    public boolean atGoal() {
        return Math.abs(RobotState.getInstance().getDistance(finalTarget)) < Constants.AutoDrive.kDistThreshold
                && Math.abs(finalTarget.getRotation().minus(RobotState.getInstance().getRobotPose().getRotation()).getRadians()) < Constants.AutoDrive.kAngleThreshold.getRadians();
    }

    public boolean invertRobotOuttake() {
        double robotAngle = RobotState.getInstance().getRobotPose().getRotation().getDegrees();
        return Math.abs(finalTarget.getRotation().getDegrees() - robotAngle) > 90;
    }

    public void stopAutoDriving() {
        if (driveToReefCommand != null)
            driveToReefCommand.cancel();

        if (driveToCoralCommand != null)
            driveToCoralCommand.cancel();
    }

    public Command close() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            isAutoDriveReefStage1 = true;
            stopAutoDriving();

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
        Constants.Swerve.kSwerveConstants.limits.maxSkidAcceleration = accLimit;

        if (driveToCoralCommand != null)
            Logger.recordOutput("Swerve/Coral Command", driveToCoralCommand.isScheduled() && !driveToCoralCommand.isFinished());
        if (driveToReefCommand != null)
            Logger.recordOutput("Swerve/Reef Command", driveToReefCommand.isScheduled() && !driveToReefCommand.isFinished());
        Logger.recordOutput("Swerve/Reef Target", target);
        Logger.recordOutput("Swerve/Acceleration Limit", accLimit);
    }
}