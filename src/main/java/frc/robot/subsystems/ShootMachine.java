package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.PositionsConstants;

import java.util.List;

import static frc.robot.subsystems.ShootMachine.ShootState.*;

public class ShootMachine extends StateMachineBase<ShootMachine.ShootState> {
    public enum ShootState {
        UNKNOWN,
        RESET,
        IDLE,
        PREPARE_HUB,
        PREPARE_DELIVERY,
        HUB,
        DELIVERY,
    }

    private ShootState state = ShootState.IDLE;
    private final Shooter shooter;
    private final Accelerator accelerator;
    private final Indexer indexer;

    public ShootMachine() {
        super(ShootState.class);
        setStateMethods(() -> state, state -> this.state = state);

        shooter = RobotContainer.getShooter();
        accelerator = RobotContainer.getAccelerator();
        indexer = RobotContainer.getIndexer();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.parallel(
            shooter.reset(),
            accelerator.reset(),
            indexer.reset()
        ));

        addEdge(RESET, IDLE);

        addEdge(List.of(IDLE, HUB), PREPARE_HUB, () -> Commands.parallel(
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.stopCmd()
        ));

        addStateCommand(PREPARE_HUB, Commands.parallel(
            shooter.autoVelocity(false)
        ));

        addEdge(PREPARE_HUB, HUB, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));

        addStateCommand(HUB, Commands.parallel(
            shooter.autoVelocity(false)
        ));

        addEdge(List.of(IDLE, DELIVERY), PREPARE_DELIVERY, () -> Commands.parallel(
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.stopCmd()
        ));

        addStateCommand(PREPARE_DELIVERY, Commands.parallel(
            shooter.autoVelocity(true)
        ));

        addEdge(PREPARE_DELIVERY, DELIVERY, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));

        addStateCommand(DELIVERY, Commands.parallel(
            shooter.autoVelocity(false)
        ));

        addEdge(PREPARE_DELIVERY, PREPARE_HUB);
        addEdge(DELIVERY, HUB);

        addStateEnd(PREPARE_HUB,
            () -> RobotState.isAutoSwitchShootReadyToShoot() && RobotState.isShootReady(),
            HUB
        );

        addStateEnd(HUB,
            () -> !RobotState.isShootReady(),
            PREPARE_HUB
        );

        addStateEnd(PREPARE_DELIVERY,
            () -> RobotState.isShootReady(),
            DELIVERY
        );

        addStateEnd(DELIVERY,
            () -> !RobotState.isDeliveryReadyWhileShooting(),
            PREPARE_DELIVERY
        );
    }
}
