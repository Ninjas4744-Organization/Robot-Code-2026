package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

import java.util.List;

public class RobotState extends RobotStateWithSwerve<States> {
    private static boolean shouldIntake = false;

    public static boolean isShouldIntake() {
        return shouldIntake;
    }

    public static void setShouldIntake(boolean shouldIntake) {
        States robotState = RobotState.getInstance().robotState;
        if (List.of(States.UNKNOWN, States.SHOOT).contains(
                robotState
        )) return;

        RobotState.shouldIntake = shouldIntake;

        States stateToChange;

        if (shouldIntake) {
            stateToChange = switch (robotState) {
                case IDLE -> States.INTAKE;
                case DELIVERY_READY -> States.INTAKE_WHILE_DELIVERY_READY;
                case DELIVERY -> States.INTAKE_WHILE_DELIVERY;
                case SHOOT_HEATED -> States.INTAKE_WHILE_SHOOT_HEATED;
                case SHOOT_READY -> States.INTAKE_WHILE_SHOOT_READY;
                case SHOOT_DYNAMIC -> States.INTAKE_WHILE_SHOOT_DYNAMIC;
                default -> null;
            };
        } else {
            stateToChange = switch (robotState) {
                case INTAKE -> States.IDLE;
                case INTAKE_WHILE_DELIVERY_READY -> States.DELIVERY_READY;
                case INTAKE_WHILE_DELIVERY -> States.DELIVERY;
                case INTAKE_WHILE_SHOOT_HEATED -> States.SHOOT_HEATED;
                case INTAKE_WHILE_SHOOT_READY -> States.SHOOT_READY;
                case INTAKE_WHILE_SHOOT_DYNAMIC -> States.SHOOT_DYNAMIC;
                default -> null;
            };
        }

        if (stateToChange != null) {
            StateMachine.getInstance().changeRobotState(stateToChange);
        }
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
