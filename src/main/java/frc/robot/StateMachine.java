package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intakealigner.IntakeAligner;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.outtake.Outtake;

import java.util.Map;

public class StateMachine extends StateMachineBase<States> {
    public StateMachine() {
        super(States.class);
    }

    public static StateMachine getInstance() {
        return (StateMachine) StateMachineBase.getInstance();
    }

    @Override
    protected void defineGraph() {
        Intake intake = RobotContainer.getIntake();
        Outtake outtake = RobotContainer.getOuttake();
        Arm arm = RobotContainer.getArm();
        IntakeAngle intakeAngle = RobotContainer.getIntakeAngle();
        IntakeAligner intakeAligner = RobotContainer.getIntakeAligner();
        Elevator elevator = RobotContainer.getElevator();
        SwerveSubsystem swerve = RobotContainer.getSwerve();

        /* **************************************** Reset **************************************** */
        addOmniEdge(States.RESET, () -> Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                arm.reset(),
                arm.setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get())),
                intake.stop(),
                intakeAligner.stop(),
                intakeAngle.reset(),
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.Close.get())),
                swerve.reset(),

                Commands.waitUntil(() -> elevator.atGoal() && arm.atGoal() && intakeAngle.atGoal())
        ));

        addEdge(States.RESET, States.IDLE);

        addStateEnd(States.RESET, Map.of(Commands.none(), States.IDLE));

        /* **************************************** Coral Intake **************************************** */
        addMultiEdge(States.INTAKE_CORAL, () -> Commands.sequence(
                intake.setVelocity(() -> -1),
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.Intake.get())),
                intakeAligner.align(),

                Commands.waitUntil(intakeAngle::atGoal)
        ), States.IDLE, States.CORAL_IN_INTAKE, States.L1_READY, States.CORAL_IN_OUTTAKE);

        addStateEnd(States.INTAKE_CORAL, Map.of(
                Commands.waitUntil(() -> intake.isCoralInside() && RobotState.getL() == 1), States.CORAL_IN_INTAKE,
                Commands.waitUntil(() -> intake.isCoralInside() && RobotState.getL() > 1), States.CORAL_IN_INTAKE
        ));

        addEdge(States.INTAKE_CORAL, States.IDLE, Commands.sequence(
                intake.setVelocity(() -> 0),
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.Close.get())),
                intakeAligner.stop()
        ));

        addEdge(States.INTAKE_CORAL, States.CORAL_IN_INTAKE, Commands.sequence(
                intake.setVelocity(() -> 0),
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.Close.get())),
                intakeAligner.stop()
        ));

        addEdge(States.CORAL_IN_INTAKE, States.CORAL_IN_OUTTAKE, Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Intake::get),
                Commands.waitUntil(elevator::atGoal),

                outtake.setPercent(() -> -1),
                intake.setVelocity(() -> 1),

                Commands.waitUntil(() -> !intake.isCoralInside()),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> outtake.forceKnowCoralInside(true)),

                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                Commands.waitUntil(elevator::atGoal)
        ));

        addEdge(States.CORAL_IN_OUTTAKE, States.CORAL_IN_INTAKE, Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Intake::get),
                Commands.waitUntil(elevator::atGoal),

                outtake.setPercent(() -> 1),
                intake.setVelocity(() -> -1),

                Commands.waitUntil(intake::isCoralInside),
                Commands.waitSeconds(0.2),
                Commands.runOnce(() -> outtake.forceKnowCoralInside(false)),

                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                Commands.waitUntil(elevator::atGoal)
        ));

        // Force know coral inside
        addEdge(States.IDLE, States.CORAL_IN_OUTTAKE);

        /* **************************************** L1 **************************************** */
        addMultiEdge(States.L1_READY, () -> Commands.sequence(
                intake.stop(),
                intakeAligner.stop(),
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.L1.get())),

                Commands.waitUntil(intakeAngle::atGoal)
        ), States.CORAL_IN_INTAKE, States.INTAKE_CORAL);

        addEdge(States.L1_READY, States.L1, Commands.sequence(
                intake.setVelocity(() -> 1)
        ));

        addStateEnd(States.L1, Map.of(
                Commands.waitSeconds(0.5), States.IDLE
        ));

        addEdge(States.L1, States.IDLE, Commands.sequence(
                intake.stop(),
                intakeAngle.setAngle(Rotation2d.fromDegrees(Constants.IntakeAngle.Positions.Close.get())),

                Commands.waitUntil(intakeAngle::atGoal)
        ));

        /* **************************************** Coral Outtake **************************************** */
        addEdge(States.CORAL_IN_OUTTAKE, States.DRIVE_REEF, Commands.sequence(

        ));

        addMultiEdge(States.L2_READY, () -> Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L3_READY, () -> Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L4_READY, () -> Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L2_INVERSE_READY, () -> Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L3_INVERSE_READY, () -> Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addMultiEdge(States.L4_INVERSE_READY, () -> Commands.sequence(

        ), States.CORAL_IN_OUTTAKE, States.DRIVE_REEF);

        addEdge(States.L2_READY, States.L2, Commands.sequence(

        ));

        addEdge(States.L3_READY, States.L3, Commands.sequence(

        ));

        addEdge(States.L4_READY, States.L4, Commands.sequence(

        ));

        addEdge(States.L2_INVERSE_READY, States.L2_INVERSE, Commands.sequence(

        ));

        addEdge(States.L3_INVERSE_READY, States.L3_INVERSE, Commands.sequence(

        ));

        addEdge(States.L4_INVERSE_READY, States.L4_INVERSE, Commands.sequence(

        ));

        addMultiEdge(States.IDLE, () -> Commands.sequence(

                ), States.L2_READY, States.L3_READY, States.L4_READY,
                States.L2, States.L3, States.L4);

        addMultiEdge(States.IDLE, () -> Commands.sequence(

                ), States.L2_INVERSE_READY, States.L3_INVERSE_READY, States.L4_INVERSE_READY,
                States.L2_INVERSE, States.L3_INVERSE, States.L4_INVERSE);

        /* **************************************** Algae Intake **************************************** */
        //region Algae Intake
        addEdge(States.IDLE, States.INTAKE_ALGAE_FLOOR, Commands.sequence(
                arm.setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeAlgaeLow.get())),
                Commands.waitUntil(arm::atGoal),
                outtake.setPercent(Constants.Outtake.Speeds.IntakeAlgae::get)
        ));

        addEdge(States.INTAKE_ALGAE_FLOOR, States.IDLE, Commands.sequence(
                outtake.stop(),
                arm.setAngle( Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get())),
                Commands.waitUntil(arm::atGoal)
        ));

        addEdge(States.INTAKE_ALGAE_FLOOR, States.ALGAE_IN_OUTTAKE, Commands.sequence(
                Commands.waitUntil(() -> outtake.isAlgaeInside())
        ));

        //TODO: USE ADD STATE END
        addStateEnd(

        );



        addEdge(States.IDLE, States.INTAKE_ALGAE_REEF, Commands.sequence(
                arm.setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeAlgaeHigh.get())),
                Commands.waitUntil(arm::atGoal),
                elevator.setHeight(() -> {
                    if (RobotState.getAlliance() == DriverStation.Alliance.Blue)
                        return Constants.Field.nearestReef().ID % 2 == 0 ? Constants.Elevator.Positions.AlgaeReefHigh.get() : Constants.Elevator.Positions.AlgaeReefLow.get();
                    return Constants.Field.nearestReef().ID % 2 == 1 ? Constants.Elevator.Positions.AlgaeReefHigh.get() : Constants.Elevator.Positions.AlgaeReefLow.get();
                }),
                outtake.setPercent(Constants.Outtake.Speeds.IntakeAlgae::get)
        ));

        addEdge(States.INTAKE_ALGAE_REEF, States.IDLE, Commands.sequence(
                arm.setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.Close.get())),
                Commands.waitUntil(arm::atGoal),
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                Commands.waitUntil(elevator::atGoal)
        ));

        addEdge(States.INTAKE_ALGAE_REEF, States.ALGAE_IN_OUTTAKE, Commands.sequence(
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal() && outtake.isAlgaeInside())
        ));
        //endregion

        /* **************************************** Algae Outtake **************************************** */
        //region Algae Outtake
        addEdge(States.ALGAE_IN_OUTTAKE, States.NET_READY, Commands.sequence(
                arm.setAngle( Rotation2d.fromDegrees(Constants.Arm.Positions.Net.get())),
                elevator.setHeight(Constants.Elevator.Positions.Net::get),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_READY, States.ALGAE_IN_OUTTAKE, Commands.sequence(
                arm.setAngle( Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeAlgaeHigh.get())),
                elevator.setHeight(Constants.Elevator.Positions.AlgaeReefHigh::get),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_READY, States.NET, Commands.sequence(
                outtake.setPercent(Constants.Outtake.Speeds.OuttakeAlgae::get),
                Commands.waitSeconds(0.2)
        ));
        //endregion

        //region Algae Outtake Mirrored
        addEdge(States.ALGAE_IN_OUTTAKE, States.NET_INVERSE_READY, Commands.sequence(
                arm.setAngle( Rotation2d.fromDegrees(Constants.Arm.Positions.NetInverse.get())),
                elevator.setHeight(Constants.Elevator.Positions.Net::get),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_INVERSE_READY, States.ALGAE_IN_OUTTAKE, Commands.sequence(
                arm.setAngle(Rotation2d.fromDegrees(Constants.Arm.Positions.IntakeAlgaeHigh.get())),
                elevator.setHeight(Constants.Elevator.Positions.AlgaeReefHigh::get),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_INVERSE_READY, States.NET_INVERSE, Commands.sequence(
                outtake.setPercent(Constants.Outtake.Speeds.OuttakeAlgae::get),
                Commands.waitSeconds(0.2)
        ));
        //endregion

        //region Close From Algae
        addEdge(States.NET, States.IDLE, Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                arm.setAngle( Rotation2d.fromRadians(Constants.Arm.Positions.Close.get())),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));

        addEdge(States.NET_INVERSE, States.IDLE, Commands.sequence(
                elevator.setHeight(Constants.Elevator.Positions.Close::get),
                arm.setAngle( Rotation2d.fromRadians(Constants.Arm.Positions.Close.get())),
                Commands.waitUntil(() -> arm.atGoal() && elevator.atGoal())
        ));
        //endregion
    }
}