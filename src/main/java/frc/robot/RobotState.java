package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

public class RobotState extends RobotStateWithSwerve<States> {
    private static States.ShootingMode shootingMode;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);

        shootingMode = States.ShootingMode.LOCK;
        setShootingMode(States.ShootingMode.LOCK);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }

    private final double predictTime = 0.5;
    private final double iterations = 20;
    public Pose3d getLookaheadTargetPose() {
        Translation2d robotVel = Swerve.getInstance().getSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation();
        Pose3d originalTarget = FieldConstants.getHubPose();
        Translation2d target = new Translation2d(originalTarget.getX(), originalTarget.getY());

        for (int i = 0; i < iterations; i++) {
            double distTarget = getDistance(new Pose2d(target, Rotation2d.kZero));
            double airTime = PositionsConstants.Shooter.getAirTime(distTarget);
            target = originalTarget.toPose2d().getTranslation().minus(robotVel.times(airTime));

            if (i == 0)
                Logger.recordOutput("Robot/Shooting/Original Lookahead Target", new Pose3d(new Translation3d(target.getX(), target.getY(), originalTarget.getZ()), originalTarget.getRotation()));
        }

        return new Pose3d(new Translation3d(target.getX(), target.getY(), originalTarget.getZ()), originalTarget.getRotation());
    }

    public double getLookaheadTargetDist() {
        return getDistance(getLookaheadTargetPose().toPose2d());
    }

    public Rotation2d getAngleToHub() {
        return getTranslation(getLookaheadTargetPose().toPose2d()).getAngle();
    }

    public static boolean isIntake() {
        return getInstance().getRobotState() == States.INTAKE
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT_HEATED
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT_READY
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT;
    }

    public static boolean isShootReady() {
        return RobotContainer.getSwerve().atGoal()
            && RobotContainer.getShooter().atGoal()
            && RobotContainer.getAccelerator().atGoal();
    }

    public static States.ShootingMode getShootingMode() {
        return shootingMode;
    }

    public static void setShootingMode(States.ShootingMode shootingMode) {
        Logger.recordOutput("Robot/Shooting/Shoot Mode", shootingMode.name());
        RobotState.shootingMode = shootingMode;
    }
}
