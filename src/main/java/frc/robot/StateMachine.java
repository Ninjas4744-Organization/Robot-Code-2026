package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.*;

import java.util.List;
import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    private SwerveSubsystem swerve;
    private Intake intake;
    private IntakeOpen intakeOpen;
    private Indexer indexer;
    private Indexer2 indexer2;
    private Shooter shooter;
    private Accelerator accelerator;
    private Climber climber;
    private ClimberAngle climberAngle;

    private BackgroundCommand shootCommand = new BackgroundCommand();

    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected void define() {
        swerve = RobotContainer.getSwerve();
        intake = RobotContainer.getIntake();
        intakeOpen = RobotContainer.getIntakeAngle();
        indexer = RobotContainer.getIndexer();
        indexer2 = RobotContainer.getIndexer2();
        shooter = RobotContainer.getShooter();
        accelerator = RobotContainer.getAccelerator();
        climber = RobotContainer.getClimber();
        climberAngle = RobotContainer.getClimberAngle();

        resetCommands();

        intakeCommands();

        shootingCommands();

        climbingCommands();
    }

    private void resetCommands() {
        addOmniEdge(States.RESET, () -> Commands.parallel(
            swerve.reset(),
            intake.reset(),
            intakeOpen.reset(),
            indexer.reset(),
            indexer2.reset(),
            shooter.reset(),
            accelerator.reset(),
            climber.reset(),
            climberAngle.reset()
        ));

        addEdge(States.RESET, States.IDLE);

        addEdge(States.STARTING_POSE, States.IDLE, Commands.sequence(
            swerve.reset(),
            indexer.stopCmd(),
            indexer2.stopCmd(),
            shooter.stopCmd(),
            accelerator.stopCmd(),
            closeIntake()
        ));

        addStateEnd(States.RESET, Map.of(Commands.waitUntil(
            () -> intake.isReset()
            && intakeOpen.isReset()
            && indexer.isReset()
            && indexer2.isReset()
            && shooter.isReset()
            && accelerator.isReset()
            && climber.isReset()
            && climberAngle.isReset()
        ), States.IDLE));
    }

    private void intakeCommands() {
        addEdge(States.IDLE, States.INTAKE, activateIntake());
        addEdge(States.INTAKE, States.IDLE, closeIntake());
    }

    private void shootingCommands() {
        addEdge(States.IDLE, States.SHOOT_HEATED, shooter.setVelocityCmd(PositionsConstants.Shooter.kShoot.get()));
        addEdge(States.SHOOT_HEATED, States.INTAKE_WHILE_SHOOT_HEATED, activateIntake());
        addEdge(States.INTAKE_WHILE_SHOOT_HEATED, States.SHOOT_HEATED, closeIntake());
        addEdge(States.INTAKE, States.INTAKE_WHILE_SHOOT_HEATED, shooter.setVelocityCmd(PositionsConstants.Shooter.kShoot.get()));
        addEdge(States.INTAKE_WHILE_SHOOT_HEATED, States.INTAKE, shooter.stopCmd());

        addEdge(List.of(States.IDLE, States.SHOOT_HEATED), States.SHOOT_READY, () -> Commands.sequence(
            activateShooting(),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get()),
            Commands.waitUntil(RobotState::isShootReady)
        ));

        addEdge(List.of(States.IDLE, States.SHOOT_HEATED, States.INTAKE, States.INTAKE_WHILE_SHOOT_HEATED), States.INTAKE_WHILE_SHOOT_READY, () -> Commands.sequence(
            activateShooting(),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get()),
            activateIntake(),
            Commands.waitUntil(RobotState::isShootReady)
        ));

        addEdge(States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY, activateIntake());
        addEdge(States.INTAKE_WHILE_SHOOT_READY, States.INTAKE, stopCmdShooting());
        addEdge(States.INTAKE_WHILE_SHOOT_READY, States.SHOOT_READY, closeIntake());

        addEdge(List.of(States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY), States.SHOOT, () -> Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            closeIntake()
        ));

        addEdge(States.SHOOT, States.SHOOT_READY, Commands.sequence(
            indexer.stopCmd(),
            indexer2.stopCmd()
        ));

        addStateCommand(States.SHOOT, Commands.either(
            Commands.parallel(
                indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
                indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndex.get())
            ),
            Commands.parallel(
                indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
                indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get())
            ),
            RobotState::isShootReady
        ).repeatedly());

        addEdge(States.INTAKE_WHILE_SHOOT, States.INTAKE_WHILE_SHOOT_READY, Commands.sequence(
            indexer.stopCmd(),
            indexer2.stopCmd()
        ));

        addEdge(List.of(States.SHOOT_HEATED,
            States.INTAKE_WHILE_SHOOT_HEATED,
            States.SHOOT,
            States.SHOOT_READY,
            States.INTAKE_WHILE_SHOOT_READY), States.IDLE, () -> Commands.sequence(
            stopCmdShooting(),
            closeIntake()
        ));

        addEdge(States.IDLE, States.DUMP, Commands.sequence(
            shooter.setVelocityCmd(PositionsConstants.Shooter.kDump.get()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndex.get()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get())
        ));

        addEdge(States.DUMP, States.IDLE, Commands.sequence(
            indexer.stopCmd(),
            indexer2.stopCmd(),
            shooter.stopCmd(),
            accelerator.stopCmd()
        ));
    }

    private void climbingCommands() {
        addEdge(States.IDLE, States.CLIMB1_READY ,Commands.sequence(
            climberAngle.setAngleCmd(Rotation2d.fromDegrees(PositionsConstants.ClimberAngle.kOpen.getAsDouble())),
            Commands.waitUntil(climberAngle::atGoal),
            climber.setPositionCmd(PositionsConstants.Climber.kClimbReady.get()),
            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB1_READY, States.CLIMB1_AUTO ,Commands.sequence(
            climber.setPositionCmd(PositionsConstants.Climber.kRightAutoClimb.get()),
            Commands.waitUntil(climber::atGoal)
        ));


        addEdge(States.CLIMB1_AUTO, States.CLIMB_DOWN ,Commands.sequence(
            climber.setPositionCmd(PositionsConstants.Climber.kLeftClimb.get()),
            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB_DOWN, States.IDLE ,Commands.sequence(
            climberAngle.setAngleCmd(Rotation2d.fromDegrees(PositionsConstants.ClimberAngle.kClose.getAsDouble())),
            Commands.waitUntil(climberAngle::atGoal)
        ));


        addEdge(States.CLIMB1_READY, States.CLIMB1 ,Commands.sequence(
            climber.setPositionCmd(PositionsConstants.Climber.kRightClimb.get()),
            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB1, States.CLIMB2 ,Commands.sequence(
            climber.setPositionCmd(PositionsConstants.Climber.kLeftClimb.get()),
            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB2, States.CLIMB3 ,Commands.sequence(
            climber.setPositionCmd(PositionsConstants.Climber.kRightClimb.get()),
            Commands.waitUntil(climber::atGoal)
        ));
    }

    private Command closeIntake() {
        return Commands.sequence(
            intake.stopCmd(),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kClose.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        );
    }

    private Command activateIntake() {
        return Commands.sequence(
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kOpen.get()),
            Commands.waitUntil(intakeOpen::atGoal),
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get())
        );
    }

    States.ShootingStates lastShootingState = null;
    private Command activateShooting() {
        return shootCommand.setNewTaskCommand(Commands.run(() -> {
            if (RobotState.getShootingState() != lastShootingState) {
                CommandScheduler.getInstance().schedule(
                    accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get())
                );

                switch (RobotState.getShootingState()) {
                    case LOCK:
                        swerve.unSlow();
                        swerve.lock();
                        shooter.autoHubVelocity();
                        break;

                    case ON_MOVE:
                        swerve.slowForShoot();
                        swerve.lookHub();
                        shooter.autoHubVelocity();
                        break;

                    case SNAP_RING:
                        swerve.unSlow();
                        swerve.snapRing();
                        shooter.autoHubVelocity();
                        break;

                    case DELIVERY:
                        swerve.unSlow();
                        swerve.delivery();
                        shooter.autoDeliveryVelocity();
                        break;
                }

                lastShootingState = RobotState.getShootingState();
            }
        }));
    }

    private Command stopCmdShooting() {
        return Commands.parallel(
            swerve.stopCmd(),
            indexer.stopCmd(),
            indexer2.stopCmd(),
            accelerator.stopCmd(),
            shooter.stopCmd(),
            Commands.runOnce(swerve::unSlow)
        );
    }
}