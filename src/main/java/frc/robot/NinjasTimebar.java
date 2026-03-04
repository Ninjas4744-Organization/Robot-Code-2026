package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class NinjasTimebar {
    private final DoublePublisher timePublisher;
    private final BooleanPublisher activePublisher;
    private final StringPublisher typePublisher;

    public NinjasTimebar(String tablePath) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(tablePath);

        // This tells Elastic which widget to use
        typePublisher = table.getStringTopic(".type").publish();
        typePublisher.set("Game Ninjas Timebar");

        timePublisher = table.getDoubleTopic("time").publish();
        activePublisher = table.getBooleanTopic("active").publish();
    }

    public void update(double time, boolean isAutoWon) {
        timePublisher.set(time);
        activePublisher.set(isAutoWon);
    }

    public void update() {
        if (DriverStation.isEnabled()) {
            if (DriverStation.isAutonomous())
                update((RobotController.getFPGATime() - Robot.autoStartTime) / 1000000, true);
            else
                update((RobotController.getFPGATime() - Robot.teleopStartTime) / 1000000 + 20, RobotState.isWonAuto());
        }
    }
}
