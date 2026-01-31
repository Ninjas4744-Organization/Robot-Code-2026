package frc.robot;

public enum States {
    UNKNOWN,
    STARTING_POSE,
    IDLE,
    RESET,

    //Intake
    INTAKE,

    //Dump & Delivery
    DUMP,
    DELIVERY_READY,
    INTAKE_WHILE_DELIVERY_READY,
    INTAKE_WHILE_DELIVERY,
    DELIVERY,

    //Shooting
    SHOOT_HEATED,
    INTAKE_WHILE_SHOOT_HEATED,
    SHOOT_READY,
    INTAKE_WHILE_SHOOT_READY,
    SHOOT,
    SHOOT_DYNAMIC,
    INTAKE_WHILE_SHOOT_DYNAMIC,

    //Climbing
    CLIMB_DOWN,
    CLIMB1_READY,
    CLIMB1_AUTO,
    CLIMB1,
    CLIMB2,
    CLIMB3
}