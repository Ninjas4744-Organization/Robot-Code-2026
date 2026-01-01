package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakealigner.IntakeAligner;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.outtake.Outtake;

import java.util.List;
import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    private Intake intake;
    private Outtake outtake;
    private Arm arm;
    private IntakeAngle intakeAngle;
    private IntakeAligner intakeAligner;
    private Elevator elevator;
    private SwerveSubsystem swerve;

    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected void defineGraph() {
        intake = RobotContainer.getIntake();
        outtake = RobotContainer.getOuttake();
        arm = RobotContainer.getArm();
        intakeAngle = RobotContainer.getIntakeAngle();
        intakeAligner = RobotContainer.getIntakeAligner();
        elevator = RobotContainer.getElevator();
        swerve = RobotContainer.getSwerve();

        resetCommands();

        intakeCommands();

        L1Commands();
        outtakeCommands();

        algaeIntakeCommands();
        algaeOuttakeCommands();
        algaeCloseCommands();
    }

    private void resetCommands() {
        addOmniEdge(States.RESET, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            arm.reset(),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            intake.stop(),
            intakeAligner.stop(),
            intakeAngle.reset(),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.CLOSE.get())),
            swerve.reset(),

            Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal() && intakeAngle.atGoal())
        ));

        addEdge(States.RESET, States.IDLE);
        addStateEnd(States.RESET, Map.of(Commands.none(), States.IDLE));

        addEdge(States.STARTING_POSE, States.IDLE, Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            swerve.reset(),

            Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal())
        ));
    }

    private void intakeCommands() {
        addMultiEdge(List.of(States.IDLE, States.CORAL_IN_INTAKE, States.L1_READY, States.CORAL_IN_OUTTAKE), States.INTAKE_CORAL, () -> Commands.sequence(
            intake.setVelocity(() -> -1),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.INTAKE.get())),
            intakeAligner.align(),
            Commands.waitUntil(intakeAngle::atGoal)
        ));

        addStateEnd(States.INTAKE_CORAL, Map.of(
            Commands.waitUntil(intake::isCoralInside), States.CORAL_IN_INTAKE
        ));

        addEdge(States.INTAKE_CORAL, States.IDLE, Commands.sequence(
            intake.setVelocity(() -> 0),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.CLOSE.get())),
            intakeAligner.stop()
        ));

        addEdge(States.INTAKE_CORAL, States.CORAL_IN_INTAKE, Commands.sequence(
            intake.setVelocity(() -> 0),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.CLOSE.get())),
            intakeAligner.stop()
        ));

        addEdge(States.CORAL_IN_INTAKE, States.CORAL_IN_OUTTAKE, Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.INTAKE.get()),
            Commands.waitUntil(elevator::atGoal),

            outtake.setPercent(() -> -1),
            intake.setVelocity(() -> 1),

            Commands.waitUntil(() -> !intake.isCoralInside()),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> outtake.forceKnowCoralInside(true)),

            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            Commands.waitUntil(elevator::atGoal)
        ));

        addEdge(States.CORAL_IN_OUTTAKE, States.CORAL_IN_INTAKE, Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.INTAKE.get()),
            Commands.waitUntil(elevator::atGoal),

            outtake.setPercent(() -> 1),
            intake.setVelocity(() -> -1),

            Commands.waitUntil(intake::isCoralInside),
            Commands.waitSeconds(0.2),
            Commands.runOnce(() -> outtake.forceKnowCoralInside(false)),

            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            Commands.waitUntil(elevator::atGoal)
        ));

        addStateEnd(States.CORAL_IN_INTAKE, Map.of(
            Commands.waitUntil(() -> RobotState.getL() > 1), States.CORAL_IN_OUTTAKE
        ));

        addStateEnd(States.CORAL_IN_OUTTAKE, Map.of(
            Commands.waitUntil(() -> RobotState.getL() == 1), States.CORAL_IN_INTAKE
        ));

        // Force know coral inside
        addEdge(States.IDLE, States.CORAL_IN_OUTTAKE);
    }

    private void L1Commands() {
        addMultiEdge(List.of(States.CORAL_IN_INTAKE, States.INTAKE_CORAL), States.L1_READY, () -> Commands.sequence(
            intake.stop(),
            intakeAligner.stop(),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.L1.get())),

            Commands.waitUntil(intakeAngle::atGoal)
        ));

        addEdge(States.L1_READY, States.L1, Commands.sequence(
            intake.setVelocity(() -> 1)
        ));

        addStateEnd(States.L1, Map.of(
            Commands.waitSeconds(0.5), States.IDLE
        ));

        addEdge(States.L1, States.IDLE, Commands.sequence(
            intake.stop(),
            intakeAngle.setAngle(Rotation2d.fromDegrees(PositionsConstants.IntakeAngle.CLOSE.get())),

            Commands.waitUntil(intakeAngle::atGoal)
        ));
    }

    private void outtakeCommands() {
        addEdge(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF, Commands.sequence(
            Commands.runOnce(() ->
                RobotState.setInverseReef(Math.abs(FieldConstants.nearestReef().pose.toPose2d().getRotation().rotateBy(Rotation2d.k180deg).minus(RobotState.getInstance().getRobotPose().getRotation()).getDegrees()) > 90)),
            new DetachedCommand(swerve.autoDriveToReef())
        ));

        addStateEnd(States.DRIVE_REEF, Map.of(
            Commands.waitUntil(() -> RobotState.getL() == 2 && !RobotState.isInverseReef()), States.L2_READY,
            Commands.waitUntil(() -> RobotState.getL() == 2 && RobotState.isInverseReef()), States.L2_INVERSE_READY,
            Commands.waitUntil(() -> RobotState.getL() == 3 && !RobotState.isInverseReef()), States.L3_READY,
            Commands.waitUntil(() -> RobotState.getL() == 3 && RobotState.isInverseReef()), States.L3_INVERSE_READY,
            Commands.waitUntil(() -> RobotState.getL() == 4 && !RobotState.isInverseReef()), States.L4_READY,
            Commands.waitUntil(() -> RobotState.getL() == 4 && RobotState.isInverseReef()), States.L4_INVERSE_READY
        ));



        /* ********** PREPARE ********** */
        addMultiEdge(List.of(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF), States.L2_READY, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.getLPosition(2, false)),
            arm.setAngle(PositionsConstants.Arm.getLPosition(2, false, false)),
            Commands.waitUntil(() -> swerve.atGoal() && elevator.atGoal() && arm.atGoal())
        ));

        addMultiEdge(List.of(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF), States.L3_READY, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.getLPosition(3, false)),
            arm.setAngle(PositionsConstants.Arm.getLPosition(3, false, false)),
            Commands.waitUntil(() -> swerve.atGoal() && elevator.atGoal() && arm.atGoal())
        ));

        addMultiEdge(List.of(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF), States.L4_READY, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.getLPosition(4, false)),
            arm.setAngle(PositionsConstants.Arm.getLPosition(4, false, false)),
            Commands.waitUntil(() -> swerve.atGoal() && elevator.atGoal() && arm.atGoal())
        ));

        addMultiEdge(List.of(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF), States.L2_INVERSE_READY, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.getLPosition(2, false)),
            arm.setAngle(PositionsConstants.Arm.getLPosition(2, true, false)),
            Commands.waitUntil(() -> swerve.atGoal() && elevator.atGoal() && arm.atGoal())
        ));

        addMultiEdge(List.of(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF), States.L3_INVERSE_READY, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.getLPosition(3, false)),
            arm.setAngle(PositionsConstants.Arm.getLPosition(3, true, false)),
            Commands.waitUntil(() -> swerve.atGoal() && elevator.atGoal() && arm.atGoal())
        ));

        addMultiEdge(List.of(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF), States.L4_INVERSE_READY, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.getLPosition(4, false)),
            arm.setAngle(PositionsConstants.Arm.getLPosition(4, true, false)),
            Commands.waitUntil(() -> swerve.atGoal() && elevator.atGoal() && arm.atGoal())
        ));



        /* ********** OUTTAKE ********** */
        addEdge(States.L2_READY, States.L2, Commands.sequence(
            arm.setAngle(PositionsConstants.Arm.getLPosition(2, false, true)),
            outtake.setPercent(() -> 1),
            Commands.waitUntil(() -> swerve.atGoal() && arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.L3_READY, States.L3, Commands.sequence(
            arm.setAngle(PositionsConstants.Arm.getLPosition(3, false, true)),
            outtake.setPercent(() -> 1),
            Commands.waitUntil(() -> swerve.atGoal() && arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.L4_READY, States.L4, Commands.sequence(
            arm.setAngle(PositionsConstants.Arm.getLPosition(4, false, true)),
            outtake.setPercent(() -> 1),
            Commands.waitUntil(() -> swerve.atGoal() && arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.L2_INVERSE_READY, States.L2_INVERSE, Commands.sequence(
            arm.setAngle(PositionsConstants.Arm.getLPosition(2, true, true)),
            outtake.setPercent(() -> 1),
            Commands.waitUntil(() -> swerve.atGoal() && arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.L3_INVERSE_READY, States.L3_INVERSE, Commands.sequence(
            arm.setAngle(PositionsConstants.Arm.getLPosition(3, true, true)),
            outtake.setPercent(() -> 1),
            Commands.waitUntil(() -> swerve.atGoal() && arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.L4_INVERSE_READY, States.L4_INVERSE, Commands.sequence(
            arm.setAngle(PositionsConstants.Arm.getLPosition(4, true, true)),
            outtake.setPercent(() -> 1),
            Commands.waitUntil(() -> swerve.atGoal() && arm.atGoal() && elevator.atGoal())
        ));



        addStateEnd(States.L2_READY, Map.of(Commands.none(), States.L2));
        addStateEnd(States.L3_READY, Map.of(Commands.none(), States.L3));
        addStateEnd(States.L4_READY, Map.of(Commands.none(), States.L4));
        addStateEnd(States.L2_INVERSE_READY, Map.of(Commands.none(), States.L2_INVERSE));
        addStateEnd(States.L3_INVERSE_READY, Map.of(Commands.none(), States.L3_INVERSE));
        addStateEnd(States.L4_INVERSE_READY, Map.of(Commands.none(), States.L4_INVERSE));
        addStateEnd(States.L2, Map.of(Commands.none(), States.IDLE));
        addStateEnd(States.L3, Map.of(Commands.none(), States.IDLE));
        addStateEnd(States.L4, Map.of(Commands.none(), States.IDLE));
        addStateEnd(States.L2_INVERSE, Map.of(Commands.none(), States.IDLE));
        addStateEnd(States.L3_INVERSE, Map.of(Commands.none(), States.IDLE));
        addStateEnd(States.L4_INVERSE, Map.of(Commands.none(), States.IDLE));



        addMultiEdge(List.of(States.L2, States.L3, States.L4,
            States.L2_INVERSE, States.L3_INVERSE, States.L4_INVERSE,
            States.L2_READY, States.L3_READY, States.L4_READY,
            States.L2_INVERSE_READY, States.L3_INVERSE_READY, States.L4_INVERSE_READY), States.IDLE, () -> Commands.sequence(
            outtake.stop(),
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            swerve.autoBackFromReef(1),
            Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal())
        ));
    }

    private void algaeIntakeCommands() {
        addEdge(States.IDLE, States.INTAKE_ALGAE_FLOOR, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.INTAKE_ALGAE_FLOOR.get())),
            Commands.waitUntil(() -> arm.getAngle().getDegrees() > -70),
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_FLOOR.get()),
            Commands.waitUntil(elevator::atGoal),
            outtake.setPercent(PositionsConstants.Outtake.INTAKE_ALGAE::get)
        ));

        addEdge(States.INTAKE_ALGAE_FLOOR, States.IDLE, Commands.sequence(
            outtake.stop(),
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            Commands.waitUntil(() -> elevator.getHeight() >= PositionsConstants.Elevator.CORAL_OUTTAKE_L3.get()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.INTAKE_ALGAE_FLOOR, States.ALGAE_IN_OUTTAKE, Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_FLOOR.get()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.ALGAE_IN_OUTTAKE.get())),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal()
            )));

        addStateEnd(States.INTAKE_ALGAE_FLOOR, Map.of(
            Commands.waitUntil(outtake::isAlgaeInside), States.ALGAE_IN_OUTTAKE
        ));

        addEdge(States.IDLE, States.INTAKE_ALGAE_REEF_LOW, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.INTAKE_ALGAE_REEF.get())),
            Commands.waitUntil(arm::atGoal),
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_REEF_LOW.get()),
            outtake.setPercent(PositionsConstants.Outtake.INTAKE_ALGAE::get)
        ));

        addEdge(States.IDLE, States.INTAKE_ALGAE_REEF_HIGH, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.INTAKE_ALGAE_REEF.get())),
            Commands.waitUntil(arm::atGoal),
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_REEF_HIGH.get()),
            outtake.setPercent(PositionsConstants.Outtake.INTAKE_ALGAE::get)
        ));

        addMultiEdge(List.of(States.INTAKE_ALGAE_REEF_LOW, States.INTAKE_ALGAE_REEF_HIGH), States.IDLE, () -> Commands.sequence(
            outtake.stop(),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal())
        ));

        addMultiEdge(List.of(States.INTAKE_ALGAE_REEF_LOW, States.INTAKE_ALGAE_REEF_HIGH), States.ALGAE_IN_OUTTAKE, () -> Commands.sequence(
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_FLOOR.get()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.ALGAE_IN_OUTTAKE.get())),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addStateEnd(States.INTAKE_ALGAE_REEF_LOW, Map.of(
            Commands.waitUntil(outtake::isAlgaeInside), States.ALGAE_IN_OUTTAKE
        ));

        addStateEnd(States.INTAKE_ALGAE_REEF_HIGH, Map.of(
            Commands.waitUntil(outtake::isAlgaeInside), States.ALGAE_IN_OUTTAKE
        ));
    }

    private void algaeOuttakeCommands() {
        addEdge(States.ALGAE_IN_OUTTAKE, States.NET_READY, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.NET.get())),
            elevator.setHeight(PositionsConstants.Elevator.NET.get()),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_READY, States.ALGAE_IN_OUTTAKE, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.ALGAE_IN_OUTTAKE.get())),
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_FLOOR.get()),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_READY, States.NET, Commands.sequence(
            outtake.setPercent(PositionsConstants.Outtake.OUTTAKE_ALGAE::get)
        ));



        addEdge(States.ALGAE_IN_OUTTAKE, States.NET_INVERSE_READY, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.NET_INVERSE.get())),
            elevator.setHeight(PositionsConstants.Elevator.NET.get()),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_INVERSE_READY, States.ALGAE_IN_OUTTAKE, Commands.sequence(
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.ALGAE_IN_OUTTAKE.get())),
            elevator.setHeight(PositionsConstants.Elevator.ALGAE_FLOOR.get()),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_INVERSE_READY, States.NET_INVERSE, Commands.sequence(
            outtake.setPercent(PositionsConstants.Outtake.OUTTAKE_ALGAE::get)
        ));
    }

    private void algaeCloseCommands() {
        addEdge(States.NET, States.IDLE, Commands.sequence(
            outtake.stop(),
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_INVERSE, States.IDLE, Commands.sequence(
            outtake.stop(),
            elevator.setHeight(PositionsConstants.Elevator.CLOSE.get()),
            arm.setAngle(Rotation2d.fromDegrees(270)),
            Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal()),
            arm.setAngle(Rotation2d.fromDegrees(PositionsConstants.Arm.CLOSE.get())),
            Commands.waitUntil(arm::atGoal)
        ));

        addStateEnd(States.NET, Map.of(
            Commands.waitSeconds(PositionsConstants.Outtake.WAIT_TIME_ALGAE_OUTTAKE.get()), States.IDLE
        ));

        addStateEnd(States.NET_INVERSE, Map.of(
            Commands.waitSeconds(PositionsConstants.Outtake.WAIT_TIME_ALGAE_OUTTAKE.get()), States.IDLE
        ));
    }
}
