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
        SAVE_OUTTAKE_BACK,
        REVERSE_BALLS,
    }

    private Shooter shooter;
    private Accelerator accelerator;
    private Indexer indexer;

    private Timer deliveryTimer;

    private ShootState stateBeforeSave = IDLE;

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

        addOmniEdge(IDLE, () -> Commands.parallel(
            shooter.stopCmd(),
            accelerator.stopCmd(),
            indexer.stopCmd()
        ));



        addEdge(List.of(IDLE, HUB), PREPARE_HUB, () -> Commands.sequence(
            indexer.stopCmd(),
            RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.LOOK_HUB)
        ));
        addStateCommand(PREPARE_HUB, Commands.sequence(
            Commands.waitUntil(() -> RobotContainer.getSwerve().getCurrentState() == SwerveSubsystem.SwerveState.LOOK_HUB && RobotContainer.getSwerve().atGoal()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            shooter.autoVelocity(false)
        ));
        addEdge(PREPARE_HUB, HUB, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));
        addStateCommand(HUB, shooter.autoVelocity(false));



        double minDelivery = 55;
        double maxDelivery = 58;
        double deliveryTime = 3;
        addEdge(List.of(IDLE, DELIVERY), PREPARE_DELIVERY, () -> Commands.sequence(
            indexer.stopCmd(),
            RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.DELIVERY)
        ));
        addStateCommand(PREPARE_DELIVERY, Commands.sequence(
            Commands.waitUntil(() -> RobotContainer.getSwerve().getCurrentState() == SwerveSubsystem.SwerveState.DELIVERY && RobotContainer.getSwerve().atGoal()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            shooter.setVelocityCmd(minDelivery)
        ));
        addEdge(PREPARE_DELIVERY, DELIVERY, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            Commands.runOnce(deliveryTimer::restart)
        ));
        addStateCommand(DELIVERY, Commands.parallel(
            Commands.run(() -> {
                shooter.setVelocity(MathUtil.clamp(maxDelivery - deliveryTimer.get() * (maxDelivery - minDelivery) / deliveryTime, minDelivery, maxDelivery));
            })
//            shooter.autoVelocity(true)
        ));



        addEdge(List.of(PREPARE_HUB, HUB, PREPARE_DELIVERY, DELIVERY, IDLE), SAVE_OUTTAKE_BACK, () -> Commands.parallel(
            accelerator.setVelocityCmd(-20),
            indexer.setVelocityCmd(-20)
        ));
        addEdge(SAVE_OUTTAKE_BACK, HUB, Commands.parallel(
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));
        addEdge(SAVE_OUTTAKE_BACK, DELIVERY, Commands.parallel(
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));



        addStateEnd(RESET, () -> true, IDLE);

        addStateEnd(PREPARE_HUB,
            () -> RobotState.isShootReady(),
            HUB
        );

//        addStateEnd(HUB,
//            () -> !RobotState.isShootReady(),
//            PREPARE_HUB
//        );

        addStateEnd(PREPARE_DELIVERY,
            () -> RobotState.isShootReady(),
            DELIVERY
        );

//        addStateEnd(DELIVERY,
//            () -> !RobotState.isShootReady(),
//            PREPARE_DELIVERY
//        );

        addStateEnd(HUB, Commands.waitUntil(() -> {
            if (accelerator.getCurrent() > 82) {
                stateBeforeSave = HUB;
                return true;
            }
            return false;
        }), SAVE_OUTTAKE_BACK);

        addStateEnd(DELIVERY, Commands.waitUntil(() -> {
            if (accelerator.getCurrent() > 82) {
                stateBeforeSave = DELIVERY;
                return true;
            }
            return false;
        }), SAVE_OUTTAKE_BACK);

        addStateEnd(SAVE_OUTTAKE_BACK, Commands.waitSeconds(0.5).andThen(Commands.waitUntil(() -> stateBeforeSave == HUB)), HUB);
        addStateEnd(SAVE_OUTTAKE_BACK, Commands.waitSeconds(0.5).andThen(Commands.waitUntil(() -> stateBeforeSave == DELIVERY)), DELIVERY);
        addStateEnd(SAVE_OUTTAKE_BACK, Commands.waitSeconds(0.5).andThen(Commands.waitUntil(() -> stateBeforeSave != HUB && stateBeforeSave != DELIVERY)), IDLE);
    }

    public void saveOuttakeBack() {
        stateBeforeSave = getCurrentState();
        changeState(SAVE_OUTTAKE_BACK);
    }
}
