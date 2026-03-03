package frc.robot;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;

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

    public void updateWithFPGATime(boolean isAutoWon) {
        timePublisher.set(Timer.getFPGATimestamp());
        activePublisher.set(isAutoWon);
    }
}
