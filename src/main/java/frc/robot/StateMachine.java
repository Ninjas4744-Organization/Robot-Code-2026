package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climberangle.ClimberAngle;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer2.Indexer2;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.shooter.Shooter;

import java.util.List;
import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    private SwerveSubsystem swerve;
    private Intake intake;
    private IntakeAngle intakeAngle;
    private Indexer indexer;
    private Indexer2 indexer2;
    private Shooter shooter;
    private Accelerator accelerator;
    private Climber climber;
    private ClimberAngle climberAngle;

    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected void defineGraph() {
        swerve = RobotContainer.getSwerve();
        intake = RobotContainer.getIntake();
        intakeAngle = RobotContainer.getIntakeAngle();
        indexer = RobotContainer.getIndexer();
        indexer2 = RobotContainer.getIndexer2();
        shooter = RobotContainer.getShooter();
        accelerator = RobotContainer.getAccelerator();
        climber = RobotContainer.getClimber();
        climberAngle = RobotContainer.getClimberAngle();

        resetCommands();

        intakeCommands();

        deliveryCommands();

        shootingCommands();

        climbingCommands();
    }

    private void resetCommands() {
        addOmniEdge(States.RESET, () -> Commands.parallel(
            swerve.reset(),
            intake.reset(),
            intakeAngle.reset(),
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
            indexer.stop(),
            indexer2.stop(),
            shooter.stop(),
            accelerator.stop(),
            closeIntake()
        ));

        addStateEnd(States.RESET, Map.of(Commands.waitUntil(
            () -> intake.isReset()
            && intakeAngle.isReset()
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

    private void deliveryCommands() {
        addEdge(States.IDLE, States.DELIVERY_READY, Commands.sequence(
                shooter.autoDeliveryVelocity(),
                Commands.waitUntil(shooter::atGoal)
        ));

        addEdge(States.DELIVERY_READY, States.DELIVERY, Commands.sequence(
                activateIndexing()
        ));

        addEdge(States.INTAKE, States.INTAKE_WHILE_DELIVERY_READY, Commands.sequence(
                shooter.autoDeliveryVelocity(),
                Commands.waitUntil(shooter::atGoal)
        ));

        addEdge(States.INTAKE_WHILE_DELIVERY_READY, States.INTAKE_WHILE_DELIVERY, Commands.sequence(
                activateIndexing()
        ));

        addEdge(States.DELIVERY, States.INTAKE_WHILE_DELIVERY, Commands.sequence(
                activateIntake()
        ));

        addEdge(States.INTAKE_WHILE_DELIVERY, States.DELIVERY, closeIntake());
        addEdge(States.INTAKE_WHILE_DELIVERY, States.INTAKE, stopShooting());

        addEdge(List.of(States.DELIVERY_READY, States.DELIVERY), States.IDLE, this::stopShooting);

        addEdge(List.of(States.INTAKE_WHILE_DELIVERY_READY, States.INTAKE_WHILE_DELIVERY), States.IDLE, () -> Commands.sequence(
                stopShooting(),
                closeIntake()
        ));

        ///--- What about SHOOT_READY -> DUMP, Eitan?
        addEdge(List.of(States.IDLE, States.SHOOT_READY), States.DUMP, () -> Commands.sequence(
                activateIndexing(),
                activateRegularShooting(PositionsConstants.Shooter.kDump.get())
        ));

        addEdge(States.DUMP, States.IDLE, stopShooting());


        addStateEnd(States.DELIVERY_READY, Map.of(Commands.none(), States.DELIVERY));
        addStateEnd(States.INTAKE_WHILE_DELIVERY_READY, Map.of(Commands.none(), States.INTAKE_WHILE_DELIVERY));
    }



    private void shootingCommands() {
        //<editor-fold desc="*********************** SHOOT HEATED **********************">
        addEdge(States.IDLE, States.SHOOT_HEATED, activateRegularShooting(PositionsConstants.Shooter.kShoot.get()));

        addEdge(States.SHOOT_HEATED, States.INTAKE_WHILE_SHOOT_HEATED, activateIntake());
        addEdge(States.INTAKE_WHILE_SHOOT_HEATED, States.SHOOT_HEATED, closeIntake());

        addEdge(States.INTAKE, States.INTAKE_WHILE_SHOOT_HEATED, activateRegularShooting(PositionsConstants.Shooter.kShoot.get()));
        addEdge(States.INTAKE_WHILE_SHOOT_HEATED, States.INTAKE, shooter.stop());
        //</editor-fold>

        //<editor-fold desc="************** SHOOT READY ******************************">
        addEdge(List.of(States.IDLE, States.SHOOT_HEATED), States.SHOOT_READY, () -> Commands.sequence(
                shooter.autoHubVelocity(),
                Commands.waitUntil(() -> shooter.atGoal())
        ));

        addEdge(List.of(States.IDLE, States.SHOOT_HEATED, States.INTAKE, States.INTAKE_WHILE_SHOOT_HEATED), States.INTAKE_WHILE_SHOOT_READY, () -> Commands.sequence(
                activateIntake(),
                shooter.autoHubVelocity(),
                Commands.waitUntil(() -> shooter.atGoal())
        ));

        addEdge(States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY, activateIntake());

        addEdge(States.INTAKE_WHILE_SHOOT_READY, States.INTAKE, shooter.stop());
        addEdge(States.INTAKE_WHILE_SHOOT_READY, States.SHOOT_READY, closeIntake());
        //</editor-fold>

        //<editor-fold desc="**************** SHOOT *****************">
        addEdge(List.of(States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY), States.SHOOT, () -> Commands.sequence(
                swerve.lock(),
                activateIndexing(),
                closeIntake()
        ));

        addEdge(List.of(States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY), States.SHOOT_DYNAMIC, () -> Commands.sequence(
                shooter.autoHubVelocity(),
                swerve.lookHub(),
                activateIndexing(),
                closeIntake()
        ));

        addEdge(List.of(States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY), States.INTAKE_WHILE_SHOOT_DYNAMIC, () -> Commands.sequence(
                activateIntake(),
                shooter.autoHubVelocity(),
                swerve.lookHub(),
                activateIndexing()
        ));

        addEdge(States.SHOOT_DYNAMIC, States.INTAKE_WHILE_SHOOT_DYNAMIC, activateIntake());
        addEdge(States.INTAKE_WHILE_SHOOT_DYNAMIC, States.SHOOT_DYNAMIC, closeIntake());


        addEdge(States.INTAKE_WHILE_SHOOT_DYNAMIC, States.INTAKE, Commands.sequence(
                shooter.stop()
        ));

        addEdge(States.INTAKE_WHILE_SHOOT_DYNAMIC, States.SHOOT_DYNAMIC, Commands.sequence(
                closeIntake()
        ));
        //</editor-fold>

        //<editor-fold desc="*************** CLOSE *************************">
        addEdge(List.of(States.SHOOT_HEATED,States.INTAKE_WHILE_SHOOT_HEATED), States.IDLE, () -> Commands.sequence(
                closeIntake(),
                shooter.stop()
        ));

        addEdge(List.of(States.SHOOT, States.SHOOT_DYNAMIC, States.INTAKE_WHILE_SHOOT_DYNAMIC, States.SHOOT_READY, States.INTAKE_WHILE_SHOOT_READY), States.IDLE, () -> Commands.sequence(
                stopShooting(),
                closeIntake()
        ));

        // Rare edge case - from INTAKE_WHILE_SHOOT -> SHOOT (While silently having intake on) -> INTAKE
        addEdge(States.SHOOT, States.INTAKE, Commands.sequence(
                stopShooting(),
                activateIntake()
        ));
        //</editor-fold>
    }

    private void climbingCommands() {
        addEdge(States.IDLE, States.CLIMB1_READY ,Commands.sequence(
            climberAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.ClimberAngle.kOpen.getAsDouble())),

            Commands.waitUntil(climberAngle::atGoal),

            climber.setPosition(PositionsConstants.Climber.kClimbReady.get()),

            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB1_READY, States.CLIMB1_AUTO ,Commands.sequence(
            climber.setPosition(PositionsConstants.Climber.kRightAutoClimb.get()),

            Commands.waitUntil(climber::atGoal)
        ));


        addEdge(States.CLIMB1_AUTO, States.CLIMB_DOWN ,Commands.sequence(
            climber.setPosition(PositionsConstants.Climber.kLeftClimb.get()),

            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB_DOWN, States.IDLE ,Commands.sequence(
            climberAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.ClimberAngle.kClose.getAsDouble())),

            Commands.waitUntil(climberAngle::atGoal)
        ));


        addEdge(States.CLIMB1_READY, States.CLIMB1 ,Commands.sequence(
            climber.setPosition(PositionsConstants.Climber.kRightClimb.get()),

            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB1, States.CLIMB2 ,Commands.sequence(
            climber.setPosition(PositionsConstants.Climber.kLeftClimb.get()),

            Commands.waitUntil(climber::atGoal)
        ));

        addEdge(States.CLIMB2, States.CLIMB3 ,Commands.sequence(
            climber.setPosition(PositionsConstants.Climber.kRightClimb.get()),

            Commands.waitUntil(climber::atGoal)
        ));
    }



    //<editor-fold desc="**************** HELPER METHODS ************************">
    private Command closeIntake() {
        return Commands.sequence(
                intake.stop(),

                intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kClose.get())),

                Commands.waitUntil(intakeAngle::atGoal)
        );
    }

    private Command activateIntake() {
        return Commands.sequence(
                intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kOpen.get())),

                Commands.waitUntil(intakeAngle::atGoal),

                // should be setVelocity after we get phoenix pro
                intake.setPercent(PositionsConstants.Intake.kIntake.get())
        );
    }
    private Command activateIndexing() {
        return Commands.sequence(
                indexer.setPercent(PositionsConstants.Indexer.kIndex.get()),
                indexer2.setPercent(PositionsConstants.Indexer.kIndex.get()),
                accelerator.setVelocity(PositionsConstants.Accelerator.kAccelerate.get())
        );
    }

    private Command stopShooting() {
        return Commands.sequence(
                indexer.stop(),
                indexer2.stop(),
                shooter.stop(),
                accelerator.stop()
        );
    }

    private Command activateRegularShooting(double velocity) {
        return Commands.sequence(
                shooter.setVelocity(velocity),
                Commands.waitUntil(shooter::atGoal)
        );
    }
    //</editor-fold>
}