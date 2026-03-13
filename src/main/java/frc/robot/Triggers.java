package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.subsystems.*;

import java.util.Set;

public class Triggers {
    private static LoggedCommandController driverController;
//    private LoggedCommandController operatorController;
    
    public static void setControllers(LoggedCommandController driverController) {
        Triggers.driverController = driverController;
    }

    public static void configureTriggers() {
        new Trigger(() -> DriverStation.isTeleop() && GeneralConstants.enableAutoTiming && RobotState.isHubAboutToBe(true, GeneralConstants.autoTimingStopDeliverySeconds)).onTrue(Commands.runOnce(() -> {
            RobotState.setShootingMode(ShootingMode.ON_MOVE);

            if (Set.of(ShootMachine.ShootState.PREPARE_HUB, ShootMachine.ShootState.PREPARE_DELIVERY, ShootMachine.ShootState.HUB, ShootMachine.ShootState.DELIVERY)
                .contains(RobotContainer.getShootMachine().getCurrentState())) {

                RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
                RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
                RobotContainer.getBox().changeStateForce(Box.BoxState.OPENED);
            }
        }));

        new Trigger(() -> DriverStation.isTeleop() && GeneralConstants.enableAutoTiming && RobotState.isHubAboutToBe(false, GeneralConstants.autoTimingSeconds)).onTrue(Commands.runOnce(() -> {
            RobotState.setShootingMode(ShootingMode.DELIVERY);

            if (Set.of(ShootMachine.ShootState.PREPARE_HUB, ShootMachine.ShootState.PREPARE_DELIVERY, ShootMachine.ShootState.HUB, ShootMachine.ShootState.DELIVERY)
                .contains(RobotContainer.getShootMachine().getCurrentState())) {

                RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
                RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
                RobotContainer.getBox().changeStateForce(Box.BoxState.CLOSED);
            }
        }));

//        new Trigger(() -> RobotState.isTeleop() && RobotContainer.getVision().isResettedPose() && Set.of(States.IDLE, States.INTAKE, States.BALLS_READY, States.DUMP).contains(RobotState.get().getRobotState()) && (RobotContainer.getSwerve().nearRightTrench() || RobotContainer.getSwerve().nearLeftTrench()))
//            .onTrue(Commands.runOnce(RobotContainer.getSwerve()::autoTrench));
//
//        new Trigger(() -> !RobotContainer.getSwerve().nearRightTrench() && !RobotContainer.getSwerve().nearLeftTrench() && Set.of(States.IDLE, States.INTAKE, States.BALLS_READY, States.DUMP).contains(RobotState.get().getRobotState()) && !StateMachine.getInstance().isTransitioning())
//            .onTrue(Commands.runOnce(RobotContainer.getSwerve()::stop));
    }

    public static void configureBindings() {
        driverController.L1().onTrue(notTest(Commands.runOnce(() -> {
            RobotContainer.getSwerve().stop();

            RobotContainer.getShootMachine().changeStateForce(ShootMachine.ShootState.IDLE);
            RobotContainer.getIntake().changeStateForce(Intake.IntakeStates.INTAKE);
            RobotContainer.getIntakeRail().changeStateForce(IntakeRail.IntakeRailState.OPENED);
            if (FieldConstants.atNeutralZone())
                RobotContainer.getBox().changeStateForce(Box.BoxState.OPENED);
            else
                RobotContainer.getBox().changeStateForce(Box.BoxState.CLOSED);
        })));

        driverController.R1().onTrue(notTest(Commands.runOnce(() -> {
            RobotContainer.getBox().changeState(Box.BoxState.OPENED);
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
                RobotContainer.getBox().changeState(Box.BoxState.SLOW_CLOSE);
            }),
            RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.SLOW_CLOSE),
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

        driverController.povUp().onTrue(notTest(Commands.runOnce(() -> {
            RobotContainer.getShootMachine().changeState(ShootMachine.ShootState.IDLE);
            RobotContainer.getIntake().changeState(Intake.IntakeStates.IDLE);
            RobotContainer.getIntakeRail().changeState(IntakeRail.IntakeRailState.CLOSED);
            RobotContainer.getBox().changeState(Box.BoxState.CLOSED);
        })));
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
            () -> RobotContainer.getBox().setPercent(0.1),
            () -> RobotContainer.getBox().setPercent(0)
        )));

        driverController.cross().whileTrue(inTest(Commands.startEnd(
            () -> RobotContainer.getBox().setPercent(-0.1),
            () -> RobotContainer.getBox().setPercent(0)
        )));

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
                RobotContainer.getAccelerator().setVelocity(20);
                RobotContainer.getIndexer().setVelocity(20);
                RobotContainer.getIntake().setVelocity(20);
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
