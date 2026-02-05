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
import org.littletonrobotics.junction.Logger;

public class RobotState extends RobotStateWithSwerve<States> {

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }

    private final double predictTime = 0.5;
    private final double iterations = 20;
    public Translation2d getHubTargetPose() {
        Translation2d robotVel = Swerve.getInstance().getSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation();
        Translation2d originalTarget = FieldConstants.getHubPose().toPose2d().getTranslation();
        Translation2d target = new Translation2d(originalTarget.getX(), originalTarget.getY());

        for (int i = 0; i < iterations; i++) {
            double distTarget = getDistance(new Pose2d(target, Rotation2d.kZero));
            double airTime = PositionsConstants.Shooter.getAirTime(distTarget);
            target = originalTarget.minus(robotVel.times(airTime));

            if (i == 0)
                Logger.recordOutput("Robot/Original Lookahead Target", new Pose2d(target, Rotation2d.kZero));
        }

        return target;
    }

    public double getDistToHub() {
        return getDistance(new Pose2d(getHubTargetPose(), Rotation2d.kZero));
    }

    public Rotation2d getAngleToHub() {
        return getTranslation(new Pose2d(getHubTargetPose(), Rotation2d.kZero)).getAngle();
    }
}
