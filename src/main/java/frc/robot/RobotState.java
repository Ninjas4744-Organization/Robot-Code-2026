package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

@SuppressWarnings("unused")
public class RobotState extends RobotStateWithSwerve<States> {
    private static States.ShootingStates shootingState = States.ShootingStates.LOCK;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }

    private final double predictTime = 0.5;

    public Translation2d getHubTargetPose() {
        Translation2d robotVel = Swerve.getInstance().getSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation();
        Translation2d originalTarget = FieldConstants.getHubPose().toPose2d().getTranslation();
        Translation2d target = new Translation2d(originalTarget.getX(), originalTarget.getY());

        double iterations = 20;
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

    public static boolean isIntake() {
        return getInstance().getRobotState() == States.INTAKE
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT_HEATED
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT_READY
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT;
    }

    public static boolean isShootReady() {
        boolean isReady = RobotContainer.getSwerve().atGoal()
            && RobotContainer.getShooter().atGoal()
            && RobotContainer.getAccelerator().atGoal();

        return RobotState.getShootingState() == States.ShootingStates.DELIVERY ?
            Math.abs(RobotState.getInstance().getRobotPose().getY() - PositionsConstants.Swerve.getDeliveryTarget().getY()) < PositionsConstants.Shooter.kDeliveryYThreshold.get() && isReady
            : isReady;
    }

    public static States.ShootingStates getShootingState() {
        return shootingState;
    }

    public static void setShootingState(States.ShootingStates shootingState) {
        RobotState.shootingState = shootingState;
    }

    public boolean isHubActive() {
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        return isHubActiveAtTime(DriverStation.getMatchTime());
    }

    public boolean isHubAboutToBeInactive(double warningSeconds) {
        if (!DriverStation.isTeleopEnabled()) return false;

        double currentTime = DriverStation.getMatchTime();
        double futureTime = currentTime - warningSeconds;

        if (futureTime < 0) return false;

        boolean activeNow = isHubActiveAtTime(currentTime);
        boolean activeFuture = isHubActiveAtTime(futureTime);

        return activeNow && !activeFuture;
    }

    private boolean isHubActiveAtTime(double matchTime) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData.isEmpty()) return true;

        boolean redInactiveFirst;

        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> { return true; }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> redInactiveFirst;
            case Blue -> !redInactiveFirst;
        };

        if (matchTime > 130) return true;
        else if (matchTime > 105) return shift1Active;
        else if (matchTime > 80) return !shift1Active;
        else if (matchTime > 55) return shift1Active;
        else if (matchTime > 30) return !shift1Active;
        else return true;
    }
}
