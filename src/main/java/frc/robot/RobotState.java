package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;

public class RobotState extends RobotStateWithSwerve<States> {
    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }

    private double feedforwardPercent = 0.7;
    public Translation2d getHubTargetPose() {
        Translation2d target = FieldConstants.getHubPose().toPose2d().getTranslation();
        double distTarget = FieldConstants.getDistToHub();

        Translation2d robotVel = Swerve.getInstance().getWantedSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation().times(feedforwardPercent)
            .plus(Swerve.getInstance().getSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation().times(1 - feedforwardPercent));
        double airTime = PositionsConstants.Shooter.getAirTime(distTarget);

        return target.minus(robotVel.times(airTime));
    }

    public Translation2d getBallEndPose() {
        Translation2d target = FieldConstants.getHubPose().toPose2d().getTranslation();
        double distTarget = FieldConstants.getDistToHub();

        Translation2d robotVel = Swerve.getInstance().getWantedSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation().times(feedforwardPercent)
            .plus(Swerve.getInstance().getSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation().times(1 - feedforwardPercent));
        double airTime = PositionsConstants.Shooter.getAirTime(distTarget);

        return target.plus(robotVel.times(airTime));
    }

    public double getDistToHub() {
        return getDistance(new Pose2d(getHubTargetPose(), Rotation2d.kZero));
    }

    public Rotation2d getAngleToHub() {
        return getTranslation(new Pose2d(getHubTargetPose(), Rotation2d.kZero)).getAngle();
    }
}
