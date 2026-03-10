package frc.robot;

public enum States {
    UNKNOWN,
    STARTING_POSE,
    RESET,
    IDLE,

    INTAKE_BOX_CLOSED,
    INTAKE_BOX_OPENED,

    SHOOT_PREPARE,
    SHOOT_READY,
    SHOOT;

    public enum ShootingMode {
        LOCK,
        ON_MOVE,
        SNAP_RING,
        DELIVERY
    }
}