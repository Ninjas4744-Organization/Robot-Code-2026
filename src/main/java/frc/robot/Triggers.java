package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;

import java.util.Set;

public class Triggers {
    private static LoggedCommandController driverController;
//    private LoggedCommandController operatorController;
    
    public static void setControllers(LoggedCommandController driverController) {
        Triggers.driverController = driverController;
    }

    public static void configureTriggers() {
        new Trigger(() -> DriverStation.isTeleop() && GeneralConstants.enableAutoTiming && RobotState.isHubAboutToBe(true, GeneralConstants.autoTimingStopDeliverySeconds)).onTrue(Commands.runOnce(() -> {
            RobotState.setShootingMode(States.ShootingMode.ON_MOVE);

            if (Set.of(States.SHOOT_PREPARE,
                    States.SHOOT_READY,
                    States.SHOOT)
                .contains(RobotState.get().getRobotState()))
                StateMachine.getInstance().changeRobotStateForce(States.INTAKE_BOX_OPENED);
        }));

        new Trigger(() -> DriverStation.isTeleop() && GeneralConstants.enableAutoTiming && RobotState.isHubAboutToBe(false, GeneralConstants.autoTimingSeconds)).onTrue(Commands.runOnce(() -> {
            RobotState.setShootingMode(States.ShootingMode.DELIVERY);

            if (Set.of(States.SHOOT_PREPARE,
                    States.SHOOT_READY,
                    States.SHOOT)
                .contains(RobotState.get().getRobotState()))
                StateMachine.getInstance().changeRobotStateForce(States.INTAKE_BOX_CLOSED);
        }));

//        new Trigger(() -> RobotState.isTeleop() && RobotContainer.getVision().isResettedPose() && Set.of(States.IDLE, States.INTAKE, States.BALLS_READY, States.DUMP).contains(RobotState.get().getRobotState()) && (RobotContainer.getSwerve().nearRightTrench() || RobotContainer.getSwerve().nearLeftTrench()))
//            .onTrue(Commands.runOnce(RobotContainer.getSwerve()::autoTrench));
//
//        new Trigger(() -> !RobotContainer.getSwerve().nearRightTrench() && !RobotContainer.getSwerve().nearLeftTrench() && Set.of(States.IDLE, States.INTAKE, States.BALLS_READY, States.DUMP).contains(RobotState.get().getRobotState()) && !StateMachine.getInstance().isTransitioning())
//            .onTrue(Commands.runOnce(RobotContainer.getSwerve()::stop));
    }

    public static void configureBindings() {
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(RobotContainer.getVision().getMegaTag1Pose() == null ? Rotation2d.kZero : RobotContainer.getVision().getMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(Rotation2d.kZero)));
        driverController.povRight().onTrue(notTest(StateMachine.getInstance().changeRobotStateForceCommand(States.RESET)));
        driverController.povUp().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.IDLE)));

        driverController.L1().onTrue(notTest(Commands.runOnce(() -> {
            RobotState.setAutoSwitchShootReadyToShoot(false);
            RobotContainer.getSwerve().stop();

            if (FieldConstants.atNeutralZone())
                StateMachine.getInstance().changeRobotStateForce(States.INTAKE_BOX_OPENED);
            else
                StateMachine.getInstance().changeRobotStateForce(States.INTAKE_BOX_CLOSED);
        })));

        driverController.R1().onTrue(notTest(Commands.runOnce(() -> {
            if (RobotState.get().getRobotState() == States.INTAKE_BOX_CLOSED)
                StateMachine.getInstance().changeRobotState(States.INTAKE_BOX_OPENED);
            else
                StateMachine.getInstance().changeRobotState(States.INTAKE_BOX_CLOSED);
        })));

        driverController.R2().onTrue(notTest(Commands.runOnce(() -> {
            if (!RobotContainer.getVision().isResettedPose())
                return;

            if (RobotState.get().getRobotState() != States.SHOOT) {
                RobotState.setAutoSwitchShootReadyToShoot(true);
                StateMachine.getInstance().changeRobotState(States.SHOOT_PREPARE);
            }
        })));

        driverController.R3().onTrue(notTest(Commands.runOnce(RobotContainer.getSwerve()::snapAngle)
            .andThen(Commands.waitUntil(RobotContainer.getSwerve()::atGoal))
            .finallyDo(RobotContainer.getSwerve()::stop)
            .onlyIf(() -> RobotContainer.getVision().isResettedPose() && Set.of(States.IDLE, States.INTAKE_BOX_CLOSED, States.INTAKE_BOX_OPENED).contains(RobotState.get().getRobotState()))));

        driverController.cross().onTrue(notTest(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.ON_MOVE))));
        driverController.square().onTrue(notTest(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.DELIVERY))));
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
            () -> RobotContainer.getIntakeOpen().setPercent(0.1),
            () -> RobotContainer.getIntakeOpen().setPercent(0)
        )));

        driverController.square().whileTrue(inTest(Commands.startEnd(
            () -> RobotContainer.getIntakeOpen().setPercent(-0.1),
            () -> RobotContainer.getIntakeOpen().setPercent(0)
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
