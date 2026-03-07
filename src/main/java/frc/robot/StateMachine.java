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

        Logger.recordOutput("State Machine/Shoot Command", shootCommand.isRunning());
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
            climberAngle.reset(),
            Commands.runOnce(() -> {
                RobotState.setIntake(false);
                RobotState.setAutoReadyToShoot(false);
            })
        )));

        addEdge(States.RESET, States.IDLE);

        addEdge(States.STARTING_POSE, States.BALLS_READY, Commands.sequence(
            swerve.reset(),
            indexer.stopCmd(),
            indexer2.stopCmd(),
            shooter.stopCmd(),
            accelerator.stopCmd(),
            intake.stopCmd(),

            intakeOpen.reset(),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kOpen.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addEdge(States.BALLS_READY, States.IDLE, Commands.sequence(
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kClose.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addEdge(States.IDLE, States.BALLS_READY, Commands.sequence(
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kOpen.get()),
            Commands.waitUntil(intakeOpen::atGoal),
            Commands.runOnce(() -> RobotState.setIntake(true))
        ));

        addStateEnd(States.RESET, () -> intake.isReset()
            && intakeOpen.isReset()
            && indexer.isReset()
            && indexer2.isReset()
            && shooter.isReset()
            && accelerator.isReset()
            && climber.isReset()
            && climberAngle.isReset(),
            States.IDLE);
    }

    private void intakeCommands() {
        addEdge(List.of(States.IDLE, States.BALLS_READY), States.INTAKE, () -> Commands.sequence(
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get()),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kOpen.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addEdge(States.INTAKE, States.BALLS_READY, intake.stopCmd());

        addEdge(States.INTAKE, States.IDLE, Commands.sequence(
            intake.stopCmd(),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kClose.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        ));


        addStateEnd(States.IDLE,
            RobotState::isIntake,
            States.INTAKE
        );

        addStateEnd(States.BALLS_READY,
            RobotState::isIntake,
            States.INTAKE
        );

        addStateEnd(States.INTAKE,
            () -> !RobotState.isIntake(),
            States.BALLS_READY
        );
    }

    private void shootingCommands() {
        addEdge(States.IDLE, States.SHOOT_HEATED, shooter.setVelocityCmd(PositionsConstants.Shooter.kShootHeat.get()));

        addEdge(List.of(States.IDLE, States.BALLS_READY, States.INTAKE, States.SHOOT_HEATED, States.SHOOT_READY, States.SHOOT), States.SHOOT_PREPARE, () -> Commands.sequence(
            activateShooting(),
//            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
//            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get()),
            indexer.stopCmd(),
            indexer2.stopCmd(),
            intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get())
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

        addEdge(States.SHOOT, States.SHOOT_READY, Commands.sequence(
//            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndexBack.get()),
//            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndexBack.get())
            indexer.stopCmd(),
            indexer2.stopCmd()
        ));

        addStateCommand(States.SHOOT, Commands.parallel(
            Commands.run(() -> {
                if (GeneralConstants.enableAutoTiming && RobotState.isHubAboutToChange(GeneralConstants.autoTimingSeconds))
                    leds.blink(Color.kRed, 0.3);
            }).finallyDo(leds::stop),

            updateIntake()
        ));

        addEdge(List.of(States.SHOOT_HEATED,
            States.SHOOT_PREPARE,
            States.SHOOT_READY,
            States.SHOOT,
            States.DUMP), States.IDLE, () -> Commands.sequence(
            stopShooting(),

            intake.stopCmd(),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kClose.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addEdge(List.of(States.SHOOT_HEATED,
            States.SHOOT_PREPARE,
            States.SHOOT_READY,
            States.SHOOT,
            States.DUMP), States.BALLS_READY, () -> Commands.sequence(
            stopShooting(),

            intake.stopCmd(),
            intakeOpen.setPositionCmd(PositionsConstants.IntakeOpen.kOpen.get()),
            Commands.waitUntil(intakeOpen::atGoal)
        ));

        addEdge(List.of(States.IDLE, States.BALLS_READY), States.DUMP, () -> Commands.sequence(
            shooter.setVelocityCmd(PositionsConstants.Shooter.kDump.get()),
            accelerator.setVelocityCmd(PositionsConstants.Accelerator.kAccelerate.get()),
            indexer.setVelocityCmd(PositionsConstants.Indexer.kIndex.get()),
            indexer2.setVelocityCmd(PositionsConstants.Indexer2.kIndex.get())
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
            () -> !RobotState.isShootReady(),
            States.SHOOT_PREPARE
        );

        addStateEnd(States.SHOOT_READY,
            RobotState::isAutoReadyToShoot,
            States.SHOOT
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


        addStateEnd(States.CLIMB1_READY,
            RobotState::isTeleop,
            States.CLIMB1
        );

        addStateEnd(States.CLIMB1_READY,
            () -> !RobotState.isTeleop(),
            States.CLIMB1_AUTO
        );

        addStateEnd(States.CLIMB1,
            () -> true,
            States.CLIMB2
        );

        addStateEnd(States.CLIMB2,
            () -> true,
            States.CLIMB3
        );

        addStateEnd(States.CLIMB_DOWN,
            () -> true,
            States.IDLE);
    }

    private States.ShootingMode lastShootMode = null;
    private boolean lastIsIntake;

    private Command updateIntake() {
        return Commands.run(() -> {
            if (RobotState.isIntake() != lastIsIntake) {
                if (RobotState.isIntake())
                    intakeOpen.setPosition(PositionsConstants.IntakeOpen.kOpen.get());
                else if (RobotState.get().getRobotState() == States.SHOOT)
                    intakeOpen.slowClose();

                lastIsIntake = RobotState.isIntake();
            }
        }).beforeStarting(() -> lastIsIntake = !RobotState.isIntake());
    }

    private Command activateShooting() {
        return shootCommand.setNewTaskCommand(Commands.sequence(
            Commands.runOnce(() -> {
                accelerator.setVelocity(PositionsConstants.Accelerator.kAccelerate.get());
                intake.setVelocityCmd(PositionsConstants.Intake.kIntake.get());
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
                shooter.autoVelocity(false);
                break;

            case ON_MOVE:
                swerve.slowForShoot();
                swerve.lookHub();
                shooter.autoVelocity(false);
                break;

            case SNAP_RING:
                swerve.unSlow();
                swerve.snapRing();
                shooter.autoVelocity(false);
                break;

            case DELIVERY:
                swerve.unSlow();
                swerve.delivery();
                shooter.autoVelocity(true);
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