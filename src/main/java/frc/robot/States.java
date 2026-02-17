package frc.robot;

public enum States {
    UNKNOWN,
    STARTING_POSE,
    IDLE,
    RESET,

    INTAKE,

    SHOOT_HEATED,
    SHOOT_PREPARE,
    SHOOT_READY,
    SHOOT,
    DUMP,

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