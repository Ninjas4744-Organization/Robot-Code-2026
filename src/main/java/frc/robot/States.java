package frc.robot;

public enum States {
     STARTING_POSE,
     IDLE,
     RESET,

     //Intake / Delivery
     INTAKE,
     INTAKE_WHILE_DELIVERY_HEATING,
     INTAKE_WHILE_DELIVERY,
     DELIVERY_HEATED,
     DELIVERY,

     //Dump
     DUMP,

     //Shooting
     SHOOT_HEATED,
     SHOOT_READY,
     SHOOT,
     INTAKE_WHILE_SHOOT_HEATED,

     //Climbing
     CLIMB1_READY,
     CLIMB1,
     CLIMB_DOWN,
     CLIMB2_READY,
     CLIMB2,
     CLIMB3_READY,
     CLIMB3
}