package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climberangle.ClimberAngle;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.intakeindexer.IntakeIndexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterindexer.ShooterIndexer;

import java.util.List;
import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    private SwerveSubsystem swerve;
    private Intake intake;
    private Shooter shooter;
    private IntakeAngle intakeAngle;
    private IntakeIndexer intakeIndexer;
    private ShooterIndexer shooterIndexer;
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
        shooter = RobotContainer.getShooter();
        intakeAngle = RobotContainer.getIntakeAngle();
        intakeIndexer = RobotContainer.getIntakeIndexer();
        shooterIndexer = RobotContainer.getShooterIndexer();
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
            intakeIndexer.reset(),
            shooterIndexer.reset(),
            shooter.reset()
        ));

        addEdge(States.RESET, States.IDLE);

        addEdge(States.STARTING_POSE, States.IDLE, Commands.sequence(
            swerve.reset(),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kClose.get()))
        ));

        addStateEnd(States.RESET, Map.of(Commands.waitUntil(
            () -> intake.isReset()
            && intakeAngle.isReset()
            && intakeIndexer.isReset()
            && shooterIndexer.isReset()
            && shooter.isReset()
        ), States.IDLE));
    }

    private void intakeCommands() {
        addEdge(States.IDLE, States.INTAKE, Commands.sequence(
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kOpen.get())),

            Commands.waitUntil(intakeAngle::atGoal),

            intake.setVelocity(PositionsConstants.Intake.kIntake.get()),
            intakeIndexer.setVelocity(PositionsConstants.IntakeIndexer.kIntake.get())
        ));

        addEdge(States.INTAKE, States.IDLE, Commands.sequence(
            intake.stop(),
            intakeIndexer.stop(),

            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kClose.get())),

            Commands.waitUntil(intakeAngle::atGoal)
        ));
    }

    private void deliveryCommands() {
        addEdge(States.IDLE, States.DELIVERY_READY, Commands.sequence(
            swerve.deliveryDrive(),
            shooter.autoDeliveryVelocity(),

            Commands.waitUntil(shooter::atGoal)
        ));

        addEdge(States.DELIVERY_READY, States.DELIVERY, Commands.sequence(
            shooterIndexer.setVelocity(PositionsConstants.ShooterIndexer.kShoot.get())
        ));

        addEdge(States.INTAKE, States.INTAKE_WHILE_DELIVERY_READY, Commands.sequence(
            swerve.deliveryDrive(),
            shooter.autoDeliveryVelocity(),

            Commands.waitUntil(shooter::atGoal)
        ));

        addEdge(States.INTAKE_WHILE_DELIVERY_READY, States.INTAKE_WHILE_DELIVERY, Commands.sequence(
            shooterIndexer.setVelocity(PositionsConstants.ShooterIndexer.kShoot.get())
        ));

        addEdge(States.DELIVERY, States.INTAKE_WHILE_DELIVERY, Commands.sequence(
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kOpen.get())),

            Commands.waitUntil(intakeAngle::atGoal),

            intake.setVelocity(PositionsConstants.Intake.kIntake.get()),
            intakeIndexer.setVelocity(PositionsConstants.IntakeIndexer.kIntake.get())
        ));

        addEdge(States.INTAKE_WHILE_DELIVERY, States.DELIVERY, Commands.sequence(
            intake.stop(),
            intakeIndexer.stop(),

            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kClose.get())),

            Commands.waitUntil(intakeAngle::atGoal)
        ));

        addEdge(List.of(States.DELIVERY_READY, States.DELIVERY), States.IDLE, () -> Commands.sequence(
            swerve.stop(),
            shooter.stop(),
            shooterIndexer.stop()
        ));

        addEdge(List.of(States.INTAKE_WHILE_DELIVERY_READY, States.INTAKE_WHILE_DELIVERY), States.IDLE, () -> Commands.sequence(
            swerve.stop(),


            shooter.stop(),
            shooterIndexer.stop(),
            intake.stop(),
            intakeIndexer.stop(),

            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kClose.get())),

            Commands.waitUntil(intakeAngle::atGoal)
        ));

        addEdge(States.IDLE, States.DUMP, Commands.sequence(
            shooter.setVelocity(PositionsConstants.Shooter.kDump.get()),
            shooterIndexer.setVelocity(PositionsConstants.ShooterIndexer.kShoot.get())
        ));

        addEdge(States.DUMP, States.IDLE, Commands.sequence(
            shooter.stop(),
            shooterIndexer.stop()
        ));

        addStateEnd(States.DELIVERY_READY, Map.of(Commands.none(), States.DELIVERY));
        addStateEnd(States.INTAKE_WHILE_DELIVERY_READY, Map.of(Commands.none(), States.INTAKE_WHILE_DELIVERY));
    }

    private void shootingCommands() {
        addEdge(States.IDLE, States.SHOOT_HEATED, Commands.sequence(
            shooter.setVelocity(PositionsConstants.Shooter.kShoot.get()),

            Commands.waitUntil(shooter::atGoal)
        ));

        addEdge(List.of(States.IDLE, States.SHOOT_HEATED), States.SHOOT_READY, () -> Commands.sequence(
            swerve.autoDrive(),
            shooter.autoHubVelocity(),

            Commands.waitUntil(() -> swerve.atGoal() && shooter.atGoal())
        ));

        addEdge(States.SHOOT_READY, States.SHOOT, Commands.sequence(
            swerve.lock(),
            shooterIndexer.setVelocity(PositionsConstants.ShooterIndexer.kShoot.get())
        ));

        addEdge(States.SHOOT_HEATED, States.INTAKE_WHILE_SHOOT_HEATED, Commands.sequence(
            intake.stop(),
            intakeIndexer.stop(),

            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.kClose.get())),

            Commands.waitUntil(intakeAngle::atGoal)
        ));

        addEdge(States.INTAKE, States.INTAKE_WHILE_SHOOT_HEATED, Commands.sequence(
            shooter.setVelocity(PositionsConstants.Shooter.kShoot.get()),

            Commands.waitUntil(shooter::atGoal)
        ));

        addEdge(States.SHOOT_HEATED, States.IDLE, Commands.sequence(
            shooter.stop()
        ));

        addEdge(List.of(States.SHOOT, States.SHOOT_READY), States.IDLE, () -> Commands.sequence(
            shooterIndexer.stop(),
            shooter.stop(),
            swerve.stop()
        ));

        addStateEnd(States.SHOOT_READY, Map.of(Commands.none(), States.SHOOT));
    }

    private void climbingCommands() {
        addEdge(States.IDLE, States.CLIMB1_READY ,Commands.sequence(
            climberAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.ClimberAngle.kOpen.getAsDouble())),

            Commands.waitUntil(climberAngle::atGoal)
        ));
        addEdge(States.CLIMB1_READY, States.CLIMB1 ,Commands.sequence(

        ));
        addEdge(States.CLIMB1, States.CLIMB_DOWN ,Commands.sequence());
        addEdge(States.CLIMB1, States.CLIMB2_READY ,Commands.sequence());
        addEdge(States.CLIMB_DOWN, States.IDLE ,Commands.sequence());
        addEdge(States.CLIMB2_READY, States.CLIMB2 ,Commands.sequence());
        addEdge(States.CLIMB2, States.CLIMB3_READY ,Commands.sequence());
        addEdge(States.CLIMB3_READY, States.CLIMB3 ,Commands.sequence());
    }
}