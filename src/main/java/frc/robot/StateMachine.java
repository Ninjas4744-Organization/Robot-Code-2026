package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;

public class StateMachine extends StateMachineBase<States> {
    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected void defineGraph() {
        addOmniEdge(States.class, States.RESET, Commands.sequence(

        ));

        /* **************************************** Coral Intake **************************************** */
        addMultiEdge(States.INTAKE_CORAL, Commands.sequence(

        ), States.IDLE, States.CORAL_IN_INTAKE, States.CORAL_IN_OUTTAKE);

        addEdge(States.INTAKE_CORAL, States.IDLE, Commands.sequence(

        ));

        addEdge(States.INTAKE_CORAL, States.CORAL_IN_INTAKE, Commands.sequence(

        ));

        addEdge(States.CORAL_IN_INTAKE, States.CORAL_IN_OUTTAKE, Commands.sequence(

        ));

        // Force know coral inside
        addEdge(States.IDLE, States.CORAL_IN_OUTTAKE);

        /* **************************************** L1 **************************************** */
        addEdge(States.CORAL_IN_INTAKE, States.L1_READY, Commands.sequence(

        ));

        addEdge(States.CORAL_IN_INTAKE, States.L1, Commands.sequence(

        ));

        addEdge(States.L1, States.IDLE, Commands.sequence(

        ));

        /* **************************************** Coral Outtake **************************************** */
        addEdge(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF, Commands.sequence(

        ));

        addMultiEdge(States.L2_READY, Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L3_READY, Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L4_READY, Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L2_INVERSE_READY, Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L3_INVERSE_READY, Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L4_INVERSE_READY, Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addEdge(States.L2_READY, States.L2, Commands.sequence(

        ));

        addEdge(States.L3_READY, States.L3, Commands.sequence(

        ));

        addEdge(States.L4_READY, States.L4, Commands.sequence(

        ));

        addEdge(States.L2_INVERSE_READY, States.L2_INVERSE, Commands.sequence(

        ));

        addEdge(States.L3_INVERSE_READY, States.L3_INVERSE, Commands.sequence(

        ));

        addEdge(States.L4_INVERSE_READY, States.L4_INVERSE, Commands.sequence(

        ));

        addMultiEdge(States.IDLE, Commands.sequence(

        ), States.L2_READY, States.L3_READY, States.L4_READY,
            States.L2, States.L3, States.L4);

        addMultiEdge(States.IDLE, Commands.sequence(

        ), States.L2_INVERSE_READY, States.L3_INVERSE_READY, States.L4_INVERSE_READY,
            States.L2_INVERSE, States.L3_INVERSE, States.L4_INVERSE);

        /* **************************************** Algae Intake **************************************** */
        addEdge(States.IDLE, States.INTAKE_ALGAE_FLOOR, Commands.sequence(

        ));

        addEdge(States.INTAKE_ALGAE_FLOOR, States.IDLE, Commands.sequence(

        ));

        addEdge(States.INTAKE_ALGAE_FLOOR, States.ALGAE_IN_OUTTAKE, Commands.sequence(

        ));

        addEdge(States.IDLE, States.INTAKE_ALGAE_REEF, Commands.sequence(

        ));

        addEdge(States.INTAKE_ALGAE_REEF, States.IDLE, Commands.sequence(

        ));

        addEdge(States.INTAKE_ALGAE_REEF, States.ALGAE_IN_OUTTAKE, Commands.sequence(

        ));

        /* **************************************** Algae Outtake **************************************** */
        addEdge(States.ALGAE_IN_OUTTAKE, States.NET_READY, Commands.sequence(

        ));

        addEdge(States.NET_READY, States.ALGAE_IN_OUTTAKE, Commands.sequence(

        ));

        addEdge(States.NET_READY, States.NET, Commands.sequence(

        ));

        addEdge(States.ALGAE_IN_OUTTAKE, States.NET_INVERSE_READY, Commands.sequence(

        ));

        addEdge(States.NET_INVERSE_READY, States.ALGAE_IN_OUTTAKE, Commands.sequence(

        ));

        addEdge(States.NET_INVERSE_READY, States.NET_INVERSE, Commands.sequence(

        ));
    }
}
