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
     DELIVERY_HEATED,
     INTAKE_WHILE_DELIVERY_HEATING,
     INTAKE_WHILE_DELIVERY,
     DELIVERY,

     //Shooting
     SHOOT_HEATED,
     INTAKE_WHILE_SHOOT_HEATED,
     SHOOT_READY,
     SHOOT,

     //Climbing
     CLIMB1_READY,
     CLIMB1,
     CLIMB_DOWN,
     CLIMB2_READY,
     CLIMB2,
     CLIMB3_READY,
     CLIMB3
}