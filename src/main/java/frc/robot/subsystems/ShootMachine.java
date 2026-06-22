package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.commands.StateEndCommand;
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



        addEdge(List.of(IDLE, REVERSE_BALLS, HUB), PREPARE_HUB, () -> Commands.sequence(
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
        addStateCommand(HUB, Commands.parallel(
            shooter.autoVelocity(false),
            Commands.sequence(
                Commands.waitSeconds(0.8),
                RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.SOFT_PUMPING)
            )
        ));



        addEdge(List.of(IDLE, REVERSE_BALLS, DELIVERY), PREPARE_DELIVERY, () -> Commands.sequence(
            indexer.stopCmd(),
            RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.DELIVERY)
        ));
        addStateCommand(PREPARE_DELIVERY, Commands.sequence(
            Commands.waitUntil(() -> RobotContainer.getSwerve().getCurrentState() == SwerveSubsystem.SwerveState.DELIVERY && RobotContainer.getSwerve().atGoal()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            shooter.autoVelocity(true)
        ));
        addEdge(PREPARE_DELIVERY, DELIVERY, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));
        addStateCommand(HUB, Commands.parallel(
            shooter.autoVelocity(true),
            Commands.sequence(
                Commands.waitSeconds(0.8),
                RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.SOFT_PUMPING)
            )
        ));



        addEdge(List.of(PREPARE_HUB, HUB, PREPARE_DELIVERY, DELIVERY, IDLE), SAVE_OUTTAKE_BACK, () -> Commands.parallel(
            accelerator.stopCmd(),
            indexer.stopCmd(),
            Commands.waitUntil(() -> indexer.getVelocity() < 20),
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



        addEdge(List.of(HUB, DELIVERY), REVERSE_BALLS, () -> Commands.sequence(
            shooter.stopCmd(),
            accelerator.stopCmd(),
//            indexer.stopCmd()
            indexer.setVelocityCmd(-20)
        ));
//        addStateCommand(REVERSE_BALLS, Commands.sequence(
//            Commands.waitUntil(() -> accelerator.getVelocity() < 30),
//            accelerator.setVelocityCmd(-20),
//            indexer.setVelocityCmd(-20)
//        ));



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

        addStateEnd(HUB, () -> {
            if (accelerator.getCurrent() > 82) {
                stateBeforeSave = HUB;
                return true;
            }
            return false;
        }, SAVE_OUTTAKE_BACK);

        addStateEnd(DELIVERY, () -> {
            if (accelerator.getCurrent() > 82) {
                stateBeforeSave = DELIVERY;
                return true;
            }
            return false;
        }, SAVE_OUTTAKE_BACK);

        addStateEnd(SAVE_OUTTAKE_BACK, new StateEndCommand(
            Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> stateBeforeSave == HUB)
        ), HUB);
        addStateEnd(SAVE_OUTTAKE_BACK, new StateEndCommand(
            Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> stateBeforeSave == DELIVERY)
        ), DELIVERY);
        addStateEnd(SAVE_OUTTAKE_BACK, new StateEndCommand(
            Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> stateBeforeSave != HUB && stateBeforeSave != DELIVERY)
        ), IDLE);

        addStateEnd(REVERSE_BALLS, new StateEndCommand(
            Commands.waitUntil(() -> accelerator.getVelocity() < 30),
            Commands.waitSeconds(1)
        ), IDLE);
    }

    public void saveOuttakeBack() {
        stateBeforeSave = getCurrentState();
        changeState(SAVE_OUTTAKE_BACK);
    }
}
