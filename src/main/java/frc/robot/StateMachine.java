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
}
