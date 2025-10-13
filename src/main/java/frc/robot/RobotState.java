package frc.robot;

import frc.lib.NinjasLib.statemachine.RobotStateBase;

public class RobotState extends RobotStateBase<States> {
    public RobotState() {
        robotState = States.IDLE;
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }
}
