package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

public class RobotState extends RobotStateWithSwerve<States> {
    private static boolean shouldIntake = false;

    public static boolean isShouldIntake() {
        return shouldIntake;
    }

    public static void setShouldIntake(boolean shouldIntake) {
        RobotState.shouldIntake = shouldIntake;
    }


    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }
}
