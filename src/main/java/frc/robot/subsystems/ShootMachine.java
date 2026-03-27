package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.constants.PositionsConstants;

import java.util.List;

import static frc.robot.subsystems.ShootMachine.ShootState.*;

public class ShootMachine extends StateMachineBase<ShootMachine.ShootState> {
    public enum ShootState {
        RESET,
        IDLE,
        PREPARE_HUB,
        PREPARE_DELIVERY,
        HUB,
        DELIVERY,
    }

    private Shooter shooter;
    private Accelerator accelerator;
    private Indexer indexer;
    private Timer deliveryTimer;

    public ShootMachine() {
        super(ShootState.class);
        currentState = IDLE;
    }

    @Override
    protected void define() {
        shooter = RobotContainer.getShooter();
        accelerator = RobotContainer.getAccelerator();
        indexer = RobotContainer.getIndexer();

        deliveryTimer = new Timer();

        addOmniEdge(RESET, () -> Commands.parallel(
            shooter.reset(),
            accelerator.reset(),
            indexer.reset()
        ));

        addEdge(RESET, IDLE);

        addEdge(List.of(IDLE, HUB), PREPARE_HUB, () -> Commands.sequence(
            indexer.stopCmd(),
            RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.LOOK_HUB),
            Commands.waitUntil(() -> RobotContainer.getSwerve().getCurrentState() == SwerveSubsystem.SwerveState.LOOK_HUB && RobotContainer.getSwerve().atGoal()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get())
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

        addEdge(List.of(IDLE, DELIVERY), PREPARE_DELIVERY, () -> Commands.sequence(
            indexer.stopCmd(),
            RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.DELIVERY),
            Commands.waitUntil(() -> RobotContainer.getSwerve().getCurrentState() == SwerveSubsystem.SwerveState.DELIVERY && RobotContainer.getSwerve().atGoal()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
//            shooter.setVelocityCmd(60)
            Commands.runOnce(deliveryTimer::restart)
        ));

//        addStateCommand(PREPARE_DELIVERY, Commands.parallel(
////            shooter.autoVelocity(true)
//            shooter.setVelocityCmd(40)
//        ));

        addEdge(PREPARE_DELIVERY, DELIVERY, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));

        double minDelivery = 55;
        double maxDelivery = 60;
        double deliveryTime = 3;
        addStateCommand(DELIVERY, Commands.parallel(
            Commands.run(() -> {
                shooter.setVelocity(MathUtil.clamp(maxDelivery - deliveryTimer.get() * (maxDelivery - minDelivery) / deliveryTime, minDelivery, maxDelivery));
            })
        ));

        addEdge(PREPARE_DELIVERY, PREPARE_HUB);
        addEdge(DELIVERY, HUB);

        addEdge(List.of(PREPARE_HUB, HUB, PREPARE_DELIVERY, DELIVERY), IDLE, () -> Commands.parallel(
            shooter.stopCmd(),
            accelerator.stopCmd(),
            indexer.stopCmd()
        ));


        addStateEnd(RESET, () -> true, IDLE);

        addStateEnd(PREPARE_HUB,
            () -> RobotState.isShootReady(),
            HUB
        );

        addStateEnd(HUB,
            () -> !RobotState.isShootReady(),
            PREPARE_HUB
        );

        addStateEnd(PREPARE_DELIVERY,
            () -> true,//RobotState.isShootReady(),
            DELIVERY
        );

//        addStateEnd(DELIVERY,
//            () -> !RobotState.isDeliveryReadyWhileShooting(),
//            PREPARE_DELIVERY
//        );
    }
}
