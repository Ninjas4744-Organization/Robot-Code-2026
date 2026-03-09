package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class RobotState extends RobotStateWithSwerve<States> {
    private static States.ShootingMode shootingMode;
    private static boolean isIntake;
    private static boolean autoSwitchShootReadyToShoot = false;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);

        shootingMode = States.ShootingMode.ON_MOVE;
        setShootingMode(States.ShootingMode.ON_MOVE);
    }

    public static RobotState get() {
        return (RobotState) RobotStateBase.get();
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
//            && RobotContainer.getAccelerator().atGoal();

        if (shootingMode == States.ShootingMode.DELIVERY) return isReady
                && (!GeneralConstants.enableAutoTiming || RobotState.isHubAboutToBe(false, GeneralConstants.autoTimingSeconds))
                && Math.abs(4 - RobotState.get().getRobotPose().getY()) > PositionsConstants.Swerve.kDeliveryYDistThreshold.get()
                && RobotState.get().getRobotPose().getX() > 4.6;
        else return isReady
                && (!GeneralConstants.enableAutoTiming || RobotState.isHubAboutToBe(true, GeneralConstants.autoTimingSeconds))
                && RobotState.get().getRobotPose().getX() < 4.6;
    }

    public static boolean isDeliveryReadyWhileShooting() {
        return (!GeneralConstants.enableAutoTiming || RobotState.isHubAboutToBe(false, GeneralConstants.autoTimingSeconds))
            && Math.abs(4 - RobotState.get().getRobotPose().getY()) > PositionsConstants.Swerve.kDeliveryYDistThreshold.get()
            && RobotState.get().getRobotPose().getX() > 4.6;
    }

    public static States.ShootingMode getShootingMode() {
        return shootingMode;
    }

    public static void setShootingMode(States.ShootingMode shootingMode) {
        Logger.recordOutput("Robot/Shooting/Shoot Mode", shootingMode.name());
        RobotState.shootingMode = shootingMode;
    }

    public static boolean isAutoSwitchShootReadyToShoot() {
        return autoSwitchShootReadyToShoot;
    }

    public static void setAutoSwitchShootReadyToShoot(boolean autoSwitchShootReadyToShoot) {
        RobotState.autoSwitchShootReadyToShoot = autoSwitchShootReadyToShoot;
        Logger.recordOutput("Robot/Shooting/Auto Switch ShootReady To Shoot", autoSwitchShootReadyToShoot);
    }

    public static double getMatchTime() {
        if (DriverStation.isAutonomousEnabled()) {
            return 160 - (RobotController.getFPGATime() - Robot.autoStartTime) / 1000000.0;
        } else if (DriverStation.isTeleopEnabled()) {
            return 140 - (RobotController.getFPGATime() - Robot.teleopStartTime) / 1000000.0;
        }
        return 0;
    }

    public static boolean isWonAuto() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return true;

        if (!GeneralConstants.kRobotMode.isSim()) {
            String gameData = DriverStation.getGameSpecificMessage();
            if (gameData.isEmpty()) return true;

            return switch (gameData.charAt(0)) {
                case 'R' -> alliance.get() == DriverStation.Alliance.Red;
                case 'B' -> alliance.get() == DriverStation.Alliance.Blue;
                default -> true;
            };
        }

        return true;
    }

    public static boolean isHubActiveAtTime(double matchTime) {
        if (matchTime > 130) return true; // Auto
        if (matchTime <= 30) return true; // End Game

        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) return true;

        boolean shift1Active = !isWonAuto();

        if (matchTime > 105) return shift1Active;      // Shift 1: 130 to 105
        else if (matchTime > 80) return !shift1Active; // Shift 2: 105 to 80
        else if (matchTime > 55) return shift1Active;  // Shift 3: 80 to 55
        else if (matchTime > 30) return !shift1Active; // Shift 4: 55 to 30
        else return true;
    }

    public static double timeUntilShiftChange() {
        if (DriverStation.isDisabled())
            return Double.POSITIVE_INFINITY;

        double matchTime = getMatchTime();

        if (matchTime > 140) return matchTime - 140;
        else if (matchTime > 130) return matchTime - 130;
        else if (matchTime > 105) return matchTime - 105;
        else if (matchTime > 80) return matchTime - 80;
        else if (matchTime > 55) return matchTime - 55;
        else if (matchTime > 30) return matchTime - 30;
        else return matchTime;
    }

    public static boolean isHubActive() {
        if (DriverStation.isAutonomousEnabled()) return true;
        if (!DriverStation.isTeleopEnabled()) return false;

        return isHubActiveAtTime(getMatchTime());
    }

    public static boolean isHubAboutToBe(boolean active, double warningSeconds) {
        if (DriverStation.isDisabled())
            return false;

        double currentTime = getMatchTime();
        double futureTime = currentTime - warningSeconds;

        if (futureTime < 0) return false;

//        boolean activeNow = isHubActiveAtTime(currentTime);
        boolean activeFuture = isHubActiveAtTime(futureTime);

        return active == activeFuture;
    }

    public static boolean isHubAboutToChange(double warningSeconds) {
        return (!isHubActive() && isHubAboutToBe(true, warningSeconds)) || (isHubActive() && isHubAboutToBe(false, warningSeconds));
    }
}
