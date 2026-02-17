package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.*;
import org.littletonrobotics.junction.Logger;

import java.util.List;

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
    private Leds leds;

    private BackgroundCommand shootCommand = new BackgroundCommand();

    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    public void periodic() {
        super.periodic();

        Logger.recordOutput("StateMachine/Shoot Command", shootCommand.isRunning());
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
        leds = RobotContainer.getLeds();

        resetCommands();

        intakeCommands();

        shootingCommands();

        climbingCommands();
    }

    private void resetCommands() {
        addOmniEdge(States.RESET, () -> new DetachedCommand(Commands.parallel(
            Commands.runOnce(shootCommand::stop),
            swerve.reset(),
            intake.reset(),
            intakeOpen.reset(),
            indexer.reset(),
            indexer2.reset(),
            shooter.reset(),
            accelerator.reset(),
            climber.reset(),
            climberAngle.reset()
        )));

        addEdge(States.RESET, States.IDLE);

        addEdge(States.STARTING_POSE, States.IDLE, Commands.sequence(
            swerve.reset(),
            indexer.stopCmd(),
            indexer2.stopCmd(),
            shooter.stopCmd(),
            accelerator.stopCmd(),
            closeIntake()
        ));

        addStateEnd(States.RESET, Commands.waitUntil(
            () -> intake.isReset()
            && intakeOpen.isReset()
            && indexer.isReset()
            && indexer2.isReset()
            && shooter.isReset()
            && accelerator.isReset()
            && climber.isReset()
            && climberAngle.isReset()
        ), States.IDLE);
    }

    private void intakeCommands() {
        addEdge(States.IDLE, States.INTAKE, activateIntake());
        addEdge(States.INTAKE, States.IDLE, closeIntake());
    }

    private void shootingCommands() {
        addEdge(States.IDLE, States.SHOOT_HEATED, shooter.setVelocityCmd(PositionsConstants.Shooter.kShoot.get()));

        addEdge(List.of(States.IDLE, States.SHOOT_HEATED, States.SHOOT_READY, States.SHOOT), States.SHOOT_PREPARE, () -> Commands.sequence(
            activateShooting(),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get())
        ));
        addStateCommand(States.SHOOT_PREPARE, updateIntake());

        addEdge(States.SHOOT_PREPARE, States.SHOOT_READY, Commands.waitUntil(RobotState::isShootReady));
        addStateCommand(States.SHOOT_READY, updateIntake());

        addEdge(States.SHOOT_READY, States.SHOOT, Commands.sequence(
            Commands.runOnce(() -> {
               if (RobotState.getShootingMode() == States.ShootingMode.SNAP_RING)
                   RobotState.setShootingMode(States.ShootingMode.LOCK);
            }),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer.kIndex.get())
        ));
        addStateCommand(States.SHOOT, updateIntake());

        addEdge(States.SHOOT, States.SHOOT_READY, Commands.sequence(
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get())
        ));

        addStateCommand(States.SHOOT, Commands.run(() -> {
            if (RobotState.isShootReady()) {
                indexer.setVelocity(PositionsConstants.Indexer.kIndex.get());
                indexer2.setVelocity(PositionsConstants.Indexer2.kIndex.get());
            } else {
                indexer.setVelocity(PositionsConstants.Indexer.kIndexBack.get());
                indexer2.setVelocity(PositionsConstants.Indexer2.kIndexBack.get());
            }

            if (GeneralConstants.enableAutoTiming && RobotState.isHubAboutToChange(3)) {
                leds.blink(Color.kRed, 0.3);
            }
        }).finallyDo(() -> {
            indexer.stop();
            indexer2.stop();
            leds.stop();
        }));

        addEdge(List.of(States.SHOOT_HEATED,
            States.SHOOT_PREPARE,
            States.SHOOT_READY,
            States.SHOOT,
            States.DUMP), States.IDLE, () -> Commands.sequence(
            stopShooting(),
            closeIntake()
        ));

        addEdge(States.IDLE, States.DUMP, Commands.sequence(
            shooter.setVelocityCmd(PositionsConstants.Shooter.kDump.get()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndex.get())
        ));

        addStateEnd(States.SHOOT,
            Commands.waitUntil(() -> GeneralConstants.enableAutoTiming && RobotState.isHubAboutToChange(3)),
            States.IDLE
        );

        addStateEnd(States.SHOOT_PREPARE,
            Commands.waitUntil(RobotState::isShootReady),
            States.SHOOT_READY
        );

        addStateEnd(States.SHOOT_READY,
            Commands.waitUntil(() -> !RobotState.isShootReady()),
            States.SHOOT_PREPARE
        );

        addStateEnd(States.SHOOT,
            Commands.waitUntil(() -> !RobotState.isShootReady()),
            States.SHOOT_PREPARE
        );
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

    private States.ShootingMode lastShootMode = null;
    private boolean lastIsIntake;

    private Command closeIntake() {
        return Commands.sequence(
            intake.stopCmd(),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kClose.get())
//            Commands.waitUntil(intakeOpen::atGoal)
        );
    }

    private Command activateIntake() {
        return Commands.sequence(
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kOpen.get()),
//            Commands.waitUntil(intakeOpen::atGoal),
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get())
        );
    }

    private Command updateIntake() {
        return Commands.run(() -> {
            if (RobotState.isIntake() != lastIsIntake) {
                if (RobotState.isIntake()) activateIntake();
                else closeIntake();
                lastIsIntake = RobotState.isIntake();
            }
        }).beforeStarting(() -> lastIsIntake = false);
    }

    private Command activateShooting() {
        return shootCommand.setNewTaskCommand(Commands.sequence(
            Commands.runOnce(() -> {
                accelerator.setVelocity(PositionsConstants.Accelerator.kAccelerate.get());
                shootSwitch();
                lastShootMode = RobotState.getShootingMode();
            }),
            Commands.run(() -> {
                if (RobotState.getShootingMode() != lastShootMode) {
                    if (getCurrentState() == States.SHOOT || getCurrentState() == States.SHOOT_READY)
                        changeRobotState(States.SHOOT_PREPARE);

                    lastShootMode = RobotState.getShootingMode();
                }
            })
        ));
    }

    private void shootSwitch() {
        switch (RobotState.getShootingMode()) {
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
    }

    private Command stopShooting() {
        return Commands.parallel(
            Commands.runOnce(shootCommand::stop),
            swerve.stopCmd(),
            indexer.stopCmd(),
            indexer2.stopCmd(),
            accelerator.stopCmd(),
            shooter.stopCmd()
        );
    }
}