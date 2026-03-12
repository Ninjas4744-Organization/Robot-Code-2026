package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.GeneralConstants;

public class NinjasTimebar {
    private final DoublePublisher timePublisher;
    private final BooleanPublisher activePublisher;
    private final StringPublisher typePublisher;

    // Flash time topics — Elastic reads these to know when to start blinking
    private final DoublePublisher activeFlashTimePublisher;
    private final DoublePublisher inactiveFlashTimePublisher;
    private final DoublePublisher endgameFlashTimePublisher;

    public NinjasTimebar(String tablePath) {
        this(tablePath, GeneralConstants.autoTimingSeconds, GeneralConstants.autoTimingStopDeliverySeconds, 10);
    }

    /**
     * @param tablePath             NT table path (e.g. "SmartDashboard/Timebar")
     * @param activeFlashTime       Seconds before end of an active shift to start blinking
     * @param inactiveFlashTime     Seconds before end of an inactive shift to start blinking
     * @param endgameFlashTime      Seconds before end of endgame to start blinking
     */
    public NinjasTimebar(String tablePath, double activeFlashTime, double inactiveFlashTime, double endgameFlashTime) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tablePath);

        // This tells Elastic which widget to use
        typePublisher = table.getStringTopic(".type").publish();
        typePublisher.set("Game Ninjas Timebar");

        timePublisher = table.getDoubleTopic("time").publish();
        activePublisher = table.getBooleanTopic("active").publish();

        activeFlashTimePublisher = table.getDoubleTopic("active_flash_time").publish();
        inactiveFlashTimePublisher = table.getDoubleTopic("inactive_flash_time").publish();
        endgameFlashTimePublisher = table.getDoubleTopic("endgame_flash_time").publish();

        activeFlashTimePublisher.set(activeFlashTime);
        inactiveFlashTimePublisher.set(inactiveFlashTime);
        endgameFlashTimePublisher.set(endgameFlashTime);
    }

    /** Manually publish time and auto-won state. */
    public void update(double time, boolean isAutoWon) {
        timePublisher.set(time);
        activePublisher.set(isAutoWon);
    }

    /** Call this in robotPeriodic() — automatically tracks match time. */
    public void update() {
        if (DriverStation.isEnabled()) {
            if (DriverStation.isAutonomous())
                update((RobotController.getFPGATime() - Robot.autoStartTime) / 1000000.0, true);
            else
                update((RobotController.getFPGATime() - Robot.teleopStartTime) / 1000000.0 + 20, RobotState.isWonAuto());
        }
    }

    /** Update flash times at runtime if needed. */
    public void setFlashTimes(double activeFlashTime, double inactiveFlashTime, double endgameFlashTime) {
        activeFlashTimePublisher.set(activeFlashTime);
        inactiveFlashTimePublisher.set(inactiveFlashTime);
        endgameFlashTimePublisher.set(endgameFlashTime);
    }
}
