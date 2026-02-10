package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.RobotStateWithSwerve;

public class RobotState extends RobotStateWithSwerve<States> {
    private static States.ShootingStates shootingState = States.ShootingStates.LOCK;

    public RobotState(SwerveDriveKinematics kinematics) {
        super(kinematics);
        robotState = States.UNKNOWN;
        setRobotState(States.UNKNOWN);
    }

    public static RobotState getInstance() {
        return (RobotState) RobotStateBase.getInstance();
    }

    public static boolean isIntake() {
        return getInstance().getRobotState() == States.INTAKE
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT_HEATED
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT_READY
            || getInstance().getRobotState() == States.INTAKE_WHILE_SHOOT;
    }

    public static boolean isShootReady() {
        return RobotContainer.getSwerve().atGoal()
            && RobotContainer.getShooter().atGoal()
            && RobotContainer.getAccelerator().atGoal();
    }

    public static States.ShootingStates getShootingState() {
        return shootingState;
    }

    public static void setShootingState(States.ShootingStates shootingState) {
        RobotState.shootingState = shootingState;
    }
}
