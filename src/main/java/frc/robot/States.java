package frc.robot;

public enum States {
    //General
    UNKNOWN,
    STARTING_POSE,
    IDLE,
    RESET,

    //Intake
    INTAKE,

    //Shooting
    SHOOT_HEATED,
    SHOOT_READY,
    SHOOT,
    INTAKE_WHILE_SHOOT_HEATED,
    INTAKE_WHILE_SHOOT_READY,
    INTAKE_WHILE_SHOOT,
    DUMP,

    //Climbing
    CLIMB_DOWN,
    CLIMB1_READY,
    CLIMB1_AUTO,
    CLIMB1,
    CLIMB2,
    CLIMB3;

    public enum ShootingMode {
        LOCK,
        ON_MOVE,
        SNAP_RING,
        DELIVERY
    }
}