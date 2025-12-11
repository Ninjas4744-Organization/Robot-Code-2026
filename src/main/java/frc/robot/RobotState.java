package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;
import org.littletonrobotics.junction.Logger;

public class RobotState extends RobotStateWithSwerve<States> {
    private static int L;
    private static boolean rightReef;
    private static boolean inverseReef;
    private static boolean inverseNet;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;

        setRobotState(States.UNKNOWN);
        setL(1);
        setReefSide(true);
        setInverseReef(false);
        setInverseNet(false);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }

    public static int getL() {
        return L;
    }

    public static void setL(int L) {
        RobotState.L = MathUtil.clamp(L, 1, 4);
        Logger.recordOutput("Reef Level", RobotState.L);
    }

    public static boolean isRightReef() {
        return rightReef;
    }

    public static void setReefSide(boolean rightReef) {
        RobotState.rightReef = rightReef;
        Logger.recordOutput("Reef Side", rightReef ? "Right" : "Left");
    }

    public static boolean isInverseReef() {
        return inverseReef;
    }

    public static void setInverseReef(boolean inverseReef) {
        RobotState.inverseReef = inverseReef;
        Logger.recordOutput("Inverse Reef", inverseReef);
    }

    public static boolean isInverseNet() {
        return inverseNet;
    }

    public static void setInverseNet(boolean inverseNet) {
        RobotState.inverseNet = inverseNet;
        Logger.recordOutput("Inverse Net", inverseNet);
    }
}
