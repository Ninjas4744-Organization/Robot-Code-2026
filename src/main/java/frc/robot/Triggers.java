package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.robot.constants.GeneralConstants;

import java.util.Set;

public class Triggers {
    private static LoggedCommandController driverController;
//    private LoggedCommandController operatorController;
    
    public static void setControllers(LoggedCommandController driverController) {
        Triggers.driverController = driverController;
    }

    public static void configureTriggers() {
        new Trigger(() -> GeneralConstants.enableAutoTiming && RobotState.isHubAboutToChange(GeneralConstants.autoTimingSeconds)).onChange(Commands.runOnce(() -> {
            if (RobotState.isHubAboutToBe(true, GeneralConstants.autoTimingSeconds))
                RobotState.setShootingMode(States.ShootingMode.ON_MOVE);
            else
                RobotState.setShootingMode(States.ShootingMode.DELIVERY);

            RobotState.setIntake(false);

            if (Set.of(States.SHOOT_HEATED,
                    States.SHOOT_PREPARE,
                    States.SHOOT_READY,
                    States.SHOOT)
                .contains(RobotState.get().getRobotState()))
                StateMachine.getInstance().changeRobotStateForce(States.BALLS_READY);
        }));

        new Trigger(() -> RobotState.isTeleop() && RobotContainer.getVision().isResettedPose() && Set.of(States.IDLE, States.INTAKE, States.BALLS_READY, States.DUMP).contains(RobotState.get().getRobotState()) && (RobotContainer.getSwerve().nearRightTrench() || RobotContainer.getSwerve().nearLeftTrench()))
            .onTrue(Commands.runOnce(RobotContainer.getSwerve()::autoTrench));

        new Trigger(() -> !RobotContainer.getSwerve().nearRightTrench() && !RobotContainer.getSwerve().nearLeftTrench() && Set.of(States.IDLE, States.INTAKE, States.BALLS_READY, States.DUMP).contains(RobotState.get().getRobotState()) && !StateMachine.getInstance().isTransitioning())
            .onTrue(Commands.runOnce(RobotContainer.getSwerve()::stop));
    }

    public static void configureBindings() {
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(RobotContainer.getVision().getMegaTag1Pose() == null ? Rotation2d.kZero : RobotContainer.getVision().getMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.get().resetGyro(Rotation2d.kZero)));
        driverController.povRight().onTrue(notTest(StateMachine.getInstance().changeRobotStateForceCommand(States.RESET)));
        driverController.povUp().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.IDLE)));

        driverController.options().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.DUMP)));

        driverController.L1().onTrue(notTest(Commands.runOnce(() -> {
            RobotState.setAutoReadyToShoot(false);
            RobotContainer.getSwerve().stop();
            RobotState.setIntake(false);
            StateMachine.getInstance().changeRobotStateForce(States.BALLS_READY);
        })));

        driverController.R1().onTrue(notTest(Commands.runOnce(() -> RobotState.setIntake(!RobotState.isIntake()))));

        driverController.R2().onTrue(notTest(Commands.runOnce(() -> {
            if (!RobotContainer.getVision().isResettedPose())
                return;

            if (RobotState.get().getRobotState() == States.SHOOT) {
                RobotState.setAutoReadyToShoot(false);
                StateMachine.getInstance().changeRobotState(States.SHOOT_READY);
            } else {
                RobotState.setAutoReadyToShoot(true);
                StateMachine.getInstance().changeRobotState(States.SHOOT_PREPARE);
            }
        })));

        driverController.L2().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.CLIMB1_READY)));

        driverController.R3().onTrue(notTest(Commands.runOnce(RobotContainer.getSwerve()::snapAngle)
            .andThen(Commands.waitUntil(RobotContainer.getSwerve()::atGoal))
            .finallyDo(RobotContainer.getSwerve()::stop)
            .onlyIf(() -> RobotContainer.getVision().isResettedPose() && Set.of(States.IDLE, States.BALLS_READY, States.INTAKE, States.DUMP).contains(RobotState.get().getRobotState()))));

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
        driverController.L2().toggleOnTrue(inTest(Commands.startEnd(() -> RobotContainer.getVision().setEnabled(false), () -> RobotContainer.getVision().setEnabled(true))));
        driverController.R2().onTrue(inTest(Commands.runOnce(() -> RobotState.get().setOdometryOnlyRobotPose(RobotContainer.getVision().getLastVisionPose()))));
    }
}
