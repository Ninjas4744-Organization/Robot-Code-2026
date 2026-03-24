package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class RobotState extends RobotStateBase {
    private static ShootingMode shootingMode;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);

        shootingMode = ShootingMode.ON_MOVE;
        setShootingMode(ShootingMode.ON_MOVE);
    }

    public static RobotState get() {
        return (RobotState) RobotStateBase.get();
    }

    public static boolean isShootReady() {
        boolean isReady = RobotContainer.getSwerve().atGoal()
            && RobotContainer.getShooter().atGoal()
            && RobotContainer.getAccelerator().atGoal();

        boolean hubActiveInTime = RobotState.isHubAboutToBe(true, GeneralConstants.autoTimingSeconds);

        boolean atHubY = Math.abs(4 - RobotState.get().getRobotPose().getY()) < PositionsConstants.Swerve.Delivery.kYDistThreshold.get();

        if (shootingMode == ShootingMode.DELIVERY)
            return isReady;
//                && (!GeneralConstants.enableAutoTiming || !hubActiveInTime)
//                && FieldConstants.atNeutralZone()
//                && !atHubY;
        else
            return isReady
                && (!GeneralConstants.enableAutoTiming || hubActiveInTime || DriverStation.isAutonomous())
                && FieldConstants.atAllianceZone();
    }

    public static boolean isDeliveryReadyWhileShooting() {
        return (!GeneralConstants.enableAutoTiming || RobotState.isHubAboutToBe(false, GeneralConstants.autoTimingSeconds))
            && Math.abs(4 - RobotState.get().getRobotPose().getY()) > PositionsConstants.Swerve.Delivery.kYDistThreshold.get()
            && RobotState.get().getRobotPose().getX() < PositionsConstants.Swerve.Delivery.kXThreshold.get()
            && FieldConstants.atNeutralZone();
    }

    public static ShootingMode getShootingMode() {
        return shootingMode;
    }

    public static void setShootingMode(ShootingMode shootingMode) {
        Logger.recordOutput("Shoot Mode", shootingMode.name());
        RobotState.shootingMode = shootingMode;
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
