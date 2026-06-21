package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeRail;
import frc.robot.subsystems.ShootMachine;
import frc.robot.subsystems.SwerveSubsystem;

public class Triggers {
    private static LoggedCommandController driverController;
    private static LoggedCommandController operatorController;
    
    public static void setControllers(LoggedCommandController driverController, LoggedCommandController operatorController) {
        Triggers.driverController = driverController;
        Triggers.operatorController = operatorController;
    }

    public static void configureTriggers() {
        // TODO: change shoot mode based on position
    }

    public static void configureBindings() {
        driverController.L1().onTrue(notTest(Commands.runOnce(() -> {
            RobotContainer.getSwerve().stop();
            RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
            RobotContainer.getIntake().changeStateForce(Intake.IntakeStates.INTAKE);
            RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
        })));

        driverController.R2().onTrue(notTest(Commands.either(
            Commands.runOnce(() -> {
                switch (RobotState.getShootingMode()) {
                    case ON_MOVE:
                        RobotContainer.getShootMachine().changeState(ShootMachine.ShootState.PREPARE_HUB);
                        break;

                    case DELIVERY:
                        RobotContainer.getShootMachine().changeState(ShootMachine.ShootState.PREPARE_DELIVERY);
                        break;
                }
                RobotContainer.getIntake().changeState(Intake.IntakeStates.INTAKE);
            }),
            Commands.sequence(
                RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.PUMPING)
            ),
            () -> !RobotContainer.getShootMachine().isInStates(ShootMachine.ShootState.HUB, ShootMachine.ShootState.PREPARE_HUB, ShootMachine.ShootState.PREPARE_DELIVERY, ShootMachine.ShootState.DELIVERY)
        )));

        driverController.R3().onTrue(notTest(
            RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.SNAP_ANGLE)
            .onlyIf(() -> RobotContainer.getShootMachine().getCurrentState() == ShootMachine.ShootState.IDLE)
        ));

        driverController.cross().onTrue(notTest(Commands.runOnce(() -> RobotState.setShootingMode(ShootingMode.ON_MOVE))));
        driverController.square().onTrue(notTest(Commands.runOnce(() -> RobotState.setShootingMode(ShootingMode.DELIVERY))));

        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(RobotContainer.getVision().getMegaTag1Pose() == null ? Rotation2d.kZero : RobotContainer.getVision().getMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(Rotation2d.kZero)));

        driverController.povRight().onTrue(notTest(Commands.runOnce(() -> RobotContainer.resetStatemachines(true))));



        operatorController.triangle().onTrue(notTest(RobotContainer.getIntakeRail().changeStateForceCommand(IntakeRail.IntakeRailState.OPENED)));
        operatorController.cross().onTrue(notTest(RobotContainer.getIntakeRail().changeStateForceCommand(IntakeRail.IntakeRailState.CLOSED)));

        operatorController.R1().onTrue(notTest(RobotContainer.getIntake().changeStateCommand(Intake.IntakeStates.INTAKE)));
        operatorController.L1().onTrue(notTest(RobotContainer.getIntake().changeStateCommand(Intake.IntakeStates.IDLE)));

        operatorController.povRight().onTrue(notTest(Commands.runOnce(() -> RobotContainer.resetStatemachines(true))));

        operatorController.circle().onTrue(Commands.runOnce(() -> RobotContainer.getShootMachine().saveOuttakeBack()));

        operatorController.square().onTrue(Commands.sequence(
            RobotContainer.getIntakeRail().changeStateForceCommand(IntakeRail.IntakeRailState.SAVE_OPEN),
            RobotContainer.getIntake().changeStateForceCommand(Intake.IntakeStates.SAVE_OUTTAKE)
        ));
    }

    private static Command inTest(Command command) {
        return command.onlyIf(DriverStation::isTest);
    }

    private static Command notTest(Command command) {
        return command.unless(DriverStation::isTest);
    }

    public static void configureTestBindings() {
        driverController.L1().toggleOnTrue(inTest(Commands.startEnd(() -> RobotContainer.getVision().setEnabled(false), () -> RobotContainer.getVision().setEnabled(true))));
        driverController.L2().onTrue(inTest(Commands.runOnce(() -> RobotState.get().setOdometryOnlyRobotPose(RobotContainer.getVision().getLastVisionPose()))));

        driverController.circle().whileTrue(inTest(Commands.startEnd(
            () -> RobotContainer.getIntakeRail().setPercent(0.5),
            () -> RobotContainer.getIntakeRail().setPercent(0)
        )));

        driverController.square().whileTrue(inTest(Commands.startEnd(
            () -> RobotContainer.getIntakeRail().setPercent(-0.5),
            () -> RobotContainer.getIntakeRail().setPercent(0)
        )));

        driverController.R2().whileTrue(inTest(Commands.startEnd(
            () -> {
                RobotContainer.getShooter().setVelocity(20);
                RobotContainer.getAccelerator().setVelocity(PositionsConstants.Accelerator.kAccelerate.get());
                RobotContainer.getIndexer().setVelocity(PositionsConstants.Indexer.kIndex.get());
                RobotContainer.getIntake().setVelocity(PositionsConstants.Intake.kIntake.get());
            },
            () -> {
                RobotContainer.getShooter()    .stop();
                RobotContainer.getAccelerator().stop();
                RobotContainer.getIndexer()    .stop();
                RobotContainer.getIntake()     .stop();
            }
        )));
    }
}
