package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.*;

import java.util.List;

public class StateMachine extends StateMachineBase<States> {
    private SwerveSubsystem swerve;
    private Intake intake;
    private IntakeOpen intakeOpen;
    private Box box;
    private Indexer indexer;
    private Shooter shooter;
    private Accelerator accelerator;
    private Leds leds;

    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    public void periodic() {
        super.periodic();
    }

    @Override
    protected void define() {
        swerve = RobotContainer.getSwerve();
        intake = RobotContainer.getIntake();
        intakeOpen = RobotContainer.getIntakeOpen();
        box = RobotContainer.getBox();
        indexer = RobotContainer.getIndexer();
        shooter = RobotContainer.getShooter();
        accelerator = RobotContainer.getAccelerator();
        leds = RobotContainer.getLeds();

        resetCommands();

        intakeCommands();

        shootingCommands();
    }

    private void resetCommands() {
        addOmniEdge(States.RESET, () -> Commands.sequence(
            Commands.runOnce(() -> {
                RobotState.setAutoSwitchShootReadyToShoot(false);
                RobotState.setShootingMode(States.ShootingMode.ON_MOVE);
            }),

            swerve.reset(),
            indexer.reset(),
            shooter.reset(),
            accelerator.reset(),

            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get()),
            box.reset(),
            intakeOpen.reset(),
            intake.reset(),
            Commands.runOnce(intakeOpen::open),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addEdge(States.RESET, States.IDLE);

        addEdge(States.STARTING_POSE, States.INTAKE_BOX_CLOSED, Commands.sequence(
            Commands.runOnce(() -> {
                RobotState.setAutoSwitchShootReadyToShoot(false);
                RobotState.setShootingMode(States.ShootingMode.ON_MOVE);
            }),

            swerve.reset(),
            indexer.stopCmd(),
            shooter.stopCmd(),
            accelerator.stopCmd(),

            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get()),
            box.reset(),
            intakeOpen.reset(),
            Commands.runOnce(intakeOpen::open),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addStateEnd(States.RESET, () -> true, States.IDLE);
    }

    private void intakeCommands() {
        addEdge(States.IDLE, States.INTAKE_BOX_CLOSED, Commands.sequence(
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get()),
            Commands.waitUntil(box::atGoal)
        ));

        addEdge(States.IDLE, States.INTAKE_BOX_OPENED, Commands.sequence(
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get()),
            box.setPositionCmd(PositionsConstants.Box.kOpen.get()),
            Commands.waitUntil(box::atGoal)
        ));

        addEdge(States.INTAKE_BOX_CLOSED, States.INTAKE_BOX_OPENED, Commands.sequence(
            box.setPositionCmd(PositionsConstants.Box.kOpen.get()),
            Commands.waitUntil(box::atGoal)
        ));

        addEdge(List.of(States.INTAKE_BOX_CLOSED, States.INTAKE_BOX_OPENED), States.IDLE, () -> Commands.sequence(
            intake.stopCmd(),
            box.setPositionCmd(PositionsConstants.Box.kClose.get()),
            Commands.waitUntil(box::atGoal)
        ));

        addStateEnd(States.INTAKE_BOX_CLOSED,
            () -> RobotState.get().getRobotPose().getX() > PositionsConstants.Swerve.kNeutralXThreshold.get(),
            States.INTAKE_BOX_OPENED
        );
    }

    private void shootingCommands() {
        addEdge(List.of(States.IDLE, States.INTAKE_BOX_CLOSED, States.INTAKE_BOX_OPENED, States.SHOOT_READY, States.SHOOT), States.SHOOT_PREPARE, () -> Commands.sequence(
            activateShooting(),
            indexer.stopCmd(),
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get())
        ));

        addEdge(States.SHOOT_PREPARE, States.SHOOT_READY, Commands.waitUntil(RobotState::isShootReady));

        addEdge(States.SHOOT_READY, States.SHOOT, Commands.sequence(
            Commands.runOnce(() -> {
               if (RobotState.getShootingMode() == States.ShootingMode.SNAP_RING)
                   RobotState.setShootingMode(States.ShootingMode.LOCK);
            }),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));

        addEdge(States.SHOOT, States.SHOOT_READY, indexer.stopCmd());

        addEdge(List.of(States.SHOOT_PREPARE, States.SHOOT_READY, States.SHOOT), States.INTAKE_BOX_CLOSED, () -> Commands.sequence(
            stopShooting()
        ));

        addEdge(List.of(States.SHOOT_PREPARE, States.SHOOT_READY, States.SHOOT), States.IDLE, () -> Commands.sequence(
            stopShooting(),
            intake.stopCmd()
        ));

        addStateEnd(States.SHOOT_PREPARE,
            RobotState::isShootReady,
            States.SHOOT_READY
        );

        addStateEnd(States.SHOOT_READY,
            () -> !RobotState.isShootReady(),
            States.SHOOT_PREPARE
        );

        addStateEnd(States.SHOOT,
            () -> RobotState.getShootingMode() != States.ShootingMode.DELIVERY ? !RobotState.isShootReady() : !RobotState.isDeliveryReadyWhileShooting(),
            States.SHOOT_PREPARE
        );

        addStateEnd(States.SHOOT,
            () -> lastShootMode != RobotState.getShootingMode(),
            States.SHOOT_PREPARE
        );

        addStateEnd(States.SHOOT_READY,
            () -> lastShootMode != RobotState.getShootingMode(),
            States.SHOOT_PREPARE
        );

        addStateEnd(States.SHOOT_READY,
            RobotState::isAutoSwitchShootReadyToShoot,
            States.SHOOT
        );
    }

    private States.ShootingMode lastShootMode = null;

    private Command activateShooting() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                accelerator.setVelocity(PositionsConstants.Accelerator.kAccelerate.get());
                intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get());
                box.slowClose();
                lastShootMode = RobotState.getShootingMode();

                switch (RobotState.getShootingMode()) {
                    case LOCK:
                        swerve.unSlow();
                        swerve.lock();
                        break;

                    case ON_MOVE:
                        swerve.slowForShoot();
                        swerve.lookHub();
                        break;

                    case SNAP_RING:
                        swerve.unSlow();
                        swerve.snapRing();
                        break;

                    case DELIVERY:
                        swerve.unSlow();
                        swerve.delivery();
                        break;
                }
            }),
            Commands.waitUntil(swerve::atGoal),
            Commands.runOnce(() -> {
                shooter.autoVelocity(RobotState.getShootingMode() == States.ShootingMode.DELIVERY);
            })
        );
    }

    private Command stopShooting() {
        return Commands.parallel(
            swerve.stopCmd(),
            indexer.stopCmd(),
            accelerator.stopCmd(),
            box.setPositionCmd(PositionsConstants.Box.kClose.get()),
            shooter.stopCmd()
        );
    }
}
