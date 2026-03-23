package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.subsystems.*;

import java.util.Set;

public class Triggers {
    private static LoggedCommandController driverController;
    private static LoggedCommandController operatorController;
    
    public static void setControllers(LoggedCommandController driverController, LoggedCommandController operatorController) {
        Triggers.driverController = driverController;
        Triggers.operatorController = operatorController;
    }

    public static void configureTriggers() {
        new Trigger(() -> DriverStation.isTeleop() && GeneralConstants.enableAutoTiming && RobotState.isHubAboutToBe(true, GeneralConstants.autoTimingStopDeliverySeconds)).onTrue(Commands.runOnce(() -> {
            RobotState.setShootingMode(ShootingMode.ON_MOVE);

            if (Set.of(ShootMachine.ShootState.PREPARE_HUB, ShootMachine.ShootState.PREPARE_DELIVERY, ShootMachine.ShootState.HUB, ShootMachine.ShootState.DELIVERY)
                .contains(RobotContainer.getShootMachine().getCurrentState())) {

                RobotContainer.getSwerve().stop();
                RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
                RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
                RobotContainer.getBox().changeStateForce(Box.BoxState.OPENED);
            }
        }));

        new Trigger(() -> DriverStation.isTeleop() && GeneralConstants.enableAutoTiming && RobotState.isHubAboutToBe(false, GeneralConstants.autoTimingSeconds)).onTrue(Commands.runOnce(() -> {
            RobotState.setShootingMode(ShootingMode.DELIVERY);

            if (Set.of(ShootMachine.ShootState.PREPARE_HUB, ShootMachine.ShootState.PREPARE_DELIVERY, ShootMachine.ShootState.HUB, ShootMachine.ShootState.DELIVERY)
                .contains(RobotContainer.getShootMachine().getCurrentState())) {

                RobotContainer.getSwerve().stop();
                RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
                RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
                RobotContainer.getBox().changeStateForce(Box.BoxState.CLOSED);
            }
        }));
    }

    public static void configureBindings() {
        driverController.L1().onTrue(notTest(Commands.runOnce(() -> {
            RobotContainer.getSwerve().stop();

            RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
            RobotContainer.getIntake().changeStateForce(Intake.IntakeStates.INTAKE);
            RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
            if (FieldConstants.atNeutralZone())
                RobotContainer.getBox().changeStateForce(Box.BoxState.OPENED);
            else if (RobotContainer.getShootMachine().getCurrentState() != ShootMachine.ShootState.IDLE)
                RobotContainer.getBox().changeStateForce(Box.BoxState.FORCE_CLOSE);
            else
                RobotContainer.getBox().changeStateForce(Box.BoxState.CLOSED);
        })));

//        driverController.R1().onTrue(notTest(Commands.runOnce(() -> {
//            RobotContainer.getBox().changeState(Box.BoxState.OPENED);
//        })));

//        driverController.L2().toggleOnTrue(notTest(Commands.startEnd(
//            () -> RobotContainer.getIntake().changeState(Intake.IntakeStates.INTAKE),
//            () -> RobotContainer.getIntake().changeState(Intake.IntakeStates.IDLE)
//        )));

        driverController.R2().onTrue(notTest(Commands.either(
            Commands.sequence(
                Commands.runOnce(() -> {
                    switch (RobotState.getShootingMode()) {
                        case ON_MOVE:
                            RobotContainer.getShootMachine().changeState(ShootMachine.ShootState.PREPARE_HUB);
                            break;

                        case DELIVERY:
                            RobotContainer.getShootMachine().changeState(ShootMachine.ShootState.PREPARE_DELIVERY);
                            break;
                    }
                }),
                Commands.either(
                    Commands.sequence(
//                        Commands.waitSeconds(1),
//                        RobotContainer.getBox().changeStateCommand(Box.BoxState.SLOW_CLOSE)
                    ),
                    Commands.none(),
                    () -> RobotState.getShootingMode() == ShootingMode.ON_MOVE
                )
            ),
            Commands.sequence(
                RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.SLOW_CLOSE)
            ),
            () -> !RobotContainer.getShootMachine().isInStates(ShootMachine.ShootState.HUB, ShootMachine.ShootState.PREPARE_HUB, ShootMachine.ShootState.PREPARE_DELIVERY, ShootMachine.ShootState.DELIVERY)
        )));

        driverController.R3().onTrue(notTest(RobotContainer.getSwerve().changeStateCommand(SwerveSubsystem.SwerveState.SNAP_ANGLE)
//            .andThen(Commands.waitUntil(RobotContainer.getSwerve()::atGoal))
//            .finallyDo(RobotContainer.getSwerve()::stop)
            .onlyIf(() -> RobotContainer.getShootMachine().getCurrentState() == ShootMachine.ShootState.IDLE)));

        driverController.cross().onTrue(notTest(Commands.runOnce(() -> RobotState.setShootingMode(ShootingMode.ON_MOVE))));
        driverController.square().onTrue(notTest(Commands.runOnce(() -> RobotState.setShootingMode(ShootingMode.DELIVERY))));

        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(RobotContainer.getVision().getMegaTag1Pose() == null ? Rotation2d.kZero : RobotContainer.getVision().getMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(Rotation2d.kZero)));

        driverController.povRight().onTrue(notTest(Commands.runOnce(RobotContainer::resetStatemachines)));

        operatorController.triangle().onTrue(notTest(RobotContainer.getBox().changeStateCommand(Box.BoxState.OPENED)));
        operatorController.cross().onTrue(notTest(RobotContainer.getBox().changeStateCommand(Box.BoxState.FORCE_CLOSE)));
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

        driverController.triangle().whileTrue(inTest(Commands.startEnd(
            () -> RobotContainer.getBox().setPercent(0.4),
            () -> RobotContainer.getBox().setPercent(0)
        )));

        driverController.cross().whileTrue(inTest(Commands.startEnd(
            () -> RobotContainer.getBox().setPercent(-0.4),
            () -> RobotContainer.getBox().setPercent(0)
        )));

        driverController.options().onTrue(inTest(RobotContainer.getBox().resetEncoderCmd()));

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
