package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    private SwerveSubsystem swerve;

    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected void defineGraph() {
        swerve = RobotContainer.getSwerve();

        resetCommands();

        intakeAndDeliveryCommands();

        shootingCommands();

        climbingCommands();
    }

    private void resetCommands() {
        addOmniEdge(States.RESET, () -> Commands.sequence(
                swerve.reset()
        ));

        addEdge(States.RESET, States.IDLE);

        addEdge(States.STARTING_POSE, States.IDLE, Commands.sequence(
                swerve.reset()
        ));

        addStateEnd(States.RESET, Map.of(Commands.none(), States.IDLE));
    }

    private void intakeAndDeliveryCommands() {
        addEdge(States.INTAKE,);
        addEdge(States.DELIVERY_HEATED,);
        addEdge(States.DELIVERY,);

        addEdge(States.INTAKE_WHILE_DELIVERY_HEATING,);
        addEdge(States.INTAKE_WHILE_DELIVERY,);

        addEdge(States.DUMP,);
    }

    private void shootingCommands() {
        addEdge(States.SHOOT_HEATED,);
        addEdge(States.SHOOT_READY,);
        addEdge(States.SHOOT,);
        addEdge(States.INTAKE_WHILE_SHOOT_HEATED,);
    }

    private void climbingCommands() {
        addEdge(States.CLIMB1_READY,);
        addEdge(States.CLIMB1,);
        addEdge(States.CLIMB_DOWN,);

        addEdge(States.CLIMB2_READY,);
        addEdge(States.CLIMB2,);
        addEdge(States.CLIMB3_READY,);
        addEdge(States.CLIMB3,);
    }
}
