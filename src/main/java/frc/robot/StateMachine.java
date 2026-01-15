package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.intakeindexer.IntakeIndexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooterindexer.ShooterIndexer;

import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    private SwerveSubsystem swerve;
    private Intake intake;
    private Shooter shooter;
    private IntakeAngle intakeAngle;
    private IntakeIndexer intakeIndexer;
    private ShooterIndexer shooterIndexer;

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
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.CLOSE.get()))
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
        addEdge(States.INTAKE,  ,Commands.sequence());
    }

    private void deliveryCommands() {
        addEdge(States.DELIVERY_HEATED,  ,Commands.sequence());
        addEdge(States.DELIVERY,  ,Commands.sequence());

        addEdge(States.INTAKE_WHILE_DELIVERY_HEATING,  ,Commands.sequence());
        addEdge(States.INTAKE_WHILE_DELIVERY,  ,Commands.sequence());

        addEdge(States.DUMP,  ,Commands.sequence());
    }

    private void shootingCommands() {
        addEdge(States.SHOOT_HEATED,  ,Commands.sequence());
        addEdge(States.SHOOT_READY,  ,Commands.sequence());
        addEdge(States.SHOOT,  ,Commands.sequence());
        addEdge(States.INTAKE_WHILE_SHOOT_HEATED,  ,Commands.sequence());
    }

    private void climbingCommands() {
        addEdge(States.CLIMB1_READY,  ,Commands.sequence());
        addEdge(States.CLIMB1,  ,Commands.sequence());
        addEdge(States.CLIMB_DOWN,  ,Commands.sequence());

        addEdge(States.CLIMB2_READY,  ,Commands.sequence());
        addEdge(States.CLIMB2,  ,Commands.sequence());
        addEdge(States.CLIMB3_READY,  ,Commands.sequence());
        addEdge(States.CLIMB3,  ,Commands.sequence());
    }
}
