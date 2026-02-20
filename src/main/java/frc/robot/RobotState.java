package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class RobotState extends RobotStateWithSwerve<States> {
    private static States.ShootingMode shootingMode;
    private static boolean isIntake;

    private static boolean autoReadyToShoot = false;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);

        shootingMode = States.ShootingMode.ON_MOVE;
        setShootingMode(States.ShootingMode.ON_MOVE);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.get();
    }

    private final double iterations = 20;
    public Pose3d getLookaheadTargetPose(Pose3d originalTarget) {
        Translation2d robotVel = Swerve.getInstance().getSpeeds().getAsFieldRelative(getRobotPose().getRotation()).toTranslation();
        Translation2d target = new Translation2d(originalTarget.getX(), originalTarget.getY());

        for (int i = 0; i < iterations; i++) {
            double distTarget = getDistance(new Pose2d(target, Rotation2d.kZero));
            double airTime = PositionsConstants.Shooter.getAirTime(distTarget);
            target = originalTarget.toPose2d().getTranslation().minus(robotVel.times(airTime));

            if (i == 0)
                Logger.recordOutput("Robot/Shooting/Original Lookahead Target", new Pose3d(new Translation3d(target.getX(), target.getY(), originalTarget.getZ()), originalTarget.getRotation()));
        }

//        target = target.minus(RobotContainer.getRobotAcceleration().times(PositionsConstants.Swerve.kAccelerationFactor.get()));

        return new Pose3d(new Translation3d(target.getX(), target.getY(), originalTarget.getZ()), originalTarget.getRotation());
    }

    public double getLookaheadTargetDist(Pose3d originalTarget) {
        return getDistance(getLookaheadTargetPose(originalTarget).toPose2d());
    }

    public Rotation2d getLookaheadTargetAngle(Pose3d originalTarget) {
        return getTranslation(getLookaheadTargetPose(originalTarget).toPose2d()).getAngle();
    }

    public static boolean isIntake() {
        return isIntake;
    }

    public static void setIntake(boolean isIntake) {
        RobotState.isIntake = isIntake;
        Logger.recordOutput("Robot/Is Intake", isIntake);
    }

    public static boolean isShootReady() {
        boolean isReady = RobotContainer.getSwerve().atGoal()
            && RobotContainer.getShooter().atGoal();
            //&& RobotContainer.getAccelerator().atGoal();

        return shootingMode == States.ShootingMode.DELIVERY
            ? isReady && RobotState.getInstance().getRobotPose().getY() > PositionsConstants.Swerve.kDeliveryYThreshold.get()
            : isReady;
    }

    public static States.ShootingMode getShootingMode() {
        return shootingMode;
    }

    public static void setShootingMode(States.ShootingMode shootingMode) {
        Logger.recordOutput("Robot/Shooting/Shoot Mode", shootingMode.name());
        RobotState.shootingMode = shootingMode;
    }


    public static boolean isAutoReadyToShoot() {
        return autoReadyToShoot;
    }

    public static void setAutoReadyToShoot(boolean autoReadyToShoot) {
        RobotState.autoReadyToShoot = autoReadyToShoot;
        Logger.recordOutput("Robot/Shooting/Auto Ready To Shoot", autoReadyToShoot);
    }

    public static double getMatchTime() {
        return 140 - (RobotController.getFPGATime() - Robot.teleopStartTime) / 1000000;
    }

    public static boolean isHubActiveAtTime(double matchTime) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return false;

        boolean shift1Active = true;
        if (!GeneralConstants.kRobotMode.isSim()) {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.isEmpty()) return true;

            boolean redInactiveFirst;

            switch (gameData.charAt(0)) {
                case 'R' -> redInactiveFirst = true;
                case 'B' -> redInactiveFirst = false;
                default -> { return true; }
            }

            shift1Active = switch (alliance.get()) {
                case Red -> redInactiveFirst;
                case Blue -> !redInactiveFirst;
            };
        }

        if (matchTime > 130) return true;
        else if (matchTime > 105) return shift1Active;
        else if (matchTime > 80) return !shift1Active;
        else if (matchTime > 55) return shift1Active;
        else if (matchTime > 30) return !shift1Active;
        else return true;
    }

    public static double timeUntilHubChange() {
        if (!DriverStation.isTeleopEnabled())
            return Double.POSITIVE_INFINITY;

        double matchTime = getMatchTime();

        if (matchTime > 130) return matchTime - 130;
        else if (matchTime > 105) return matchTime - 105;
        else if (matchTime > 80) return matchTime - 80;
        else if (matchTime > 55) return matchTime - 55;
        else if (matchTime > 30) return matchTime - 30;
        else return Double.POSITIVE_INFINITY;
    }

    public static boolean isHubActive() {
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        return isHubActiveAtTime(getMatchTime());
    }

    public static boolean isHubAboutToBe(boolean active, double warningSeconds) {
        if (!DriverStation.isTeleopEnabled())
            return false;

        double currentTime = getMatchTime();
        double futureTime = currentTime - warningSeconds;

        if (futureTime < 0) return false;

        boolean activeNow = isHubActiveAtTime(currentTime);
        boolean activeFuture = isHubActiveAtTime(futureTime);

        return active ? !activeNow && activeFuture : activeNow && !activeFuture;
    }

    public static boolean isHubAboutToChange(double warningSeconds) {
        return isHubAboutToBe(true, warningSeconds) || isHubAboutToBe(false, warningSeconds);
    }
}
