package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private static SwerveSubsystem swerveSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static Intake intake;
    private static IntakeRail intakeRail;
    private static Box box;
    private static Indexer indexer;
    private static Shooter shooter;
    private static Accelerator accelerator;
    private static ShootMachine shootMachine;
    private static Leds leds;

    private LoggedCommandController driverController;
    private LoggedDashboardChooser<Command> autoChooser;
    private ShootCalculator shootCalculator;
    private NinjasTimebar timebar;
    private Field2d logField = new Field2d();

    public RobotContainer() {
        if (!GeneralConstants.kRobotMode.isReplay())
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
        else
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});

        swerveSubsystem = new SwerveSubsystem(true, false, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, driverController::getRightY);
        RobotStateBase.set(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        visionSubsystem = new VisionSubsystem();
        shootCalculator = new ShootCalculator();

        intake = new Intake(true);
        intakeRail = new IntakeRail(true);
        box = new Box(true);
        indexer = new Indexer(true);
        shooter = new Shooter(true);
        accelerator = new Accelerator(true);
        leds = new Leds(false);

        shootMachine = new ShootMachine();

        configureAuto();
        Triggers.setControllers(driverController);
        Triggers.configureBindings();
        Triggers.configureTestBindings();
        Triggers.configureTriggers();

        if (GeneralConstants.kRobotMode.isSim()) {
            Simulation.setup();
        }

        timebar = new NinjasTimebar("Timebar");
    }

    public static SwerveSubsystem getSwerve() { return swerveSubsystem; }
    public static VisionSubsystem getVision() { return visionSubsystem; }
    public static Intake getIntake() { return intake; }
    public static IntakeRail getIntakeRail() { return intakeRail; }
    public static Box getBox() { return box; }
    public static Indexer getIndexer() { return indexer; }
    public static Shooter getShooter() { return shooter; }
    public static Accelerator getAccelerator() { return accelerator; }
    public static ShootMachine getShootMachine() { return shootMachine; }
    public static Leds getLeds() { return leds; }

    private void configureAuto() {
        AutoBuilder.configure(
            RobotState.get()::getRobotPose,
            RobotState.get()::setRobotPose,
            Swerve.getInstance()::getSpeeds,
            swerveSubsystem::setAutoInput,
            SubsystemConstants.kAutonomyConfig,
            SubsystemConstants.kSwerve.special.robotConfig,
            () -> false
        );

        NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> {
            shootMachine.changeState(ShootMachine.ShootState.PREPARE_HUB);
            RobotContainer.getBox().changeState(Box.BoxState.SLOW_CLOSE);
            RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.SLOW_CLOSE);
        }));

        NamedCommands.registerCommand("Stop", Commands.runOnce(() -> {
            shootMachine.changeState(ShootMachine.ShootState.IDLE);
            swerveSubsystem.stop();
        }));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        SwerveSpeeds robotVel = Swerve.getInstance().getSpeeds();

        Logger.recordOutput("Robot/Speed/Robot Speed", robotVel.getSpeed());
        Logger.recordOutput("Robot/Speed/Robot Acceleration", swerveSubsystem.getAcceleration());

        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision", visionSubsystem.getMegaTag1Pose());
        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision Dist", visionSubsystem.getMegaTag1DistFromTag());
        Logger.recordOutput("Robot/Vision/Odometry Only Pose", RobotState.get().getOdometryOnlyRobotPose());
        Logger.recordOutput("Robot/Vision/Odometry Vision Error", visionSubsystem.getLastVisionPose().getTranslation().getDistance(RobotState.get().getOdometryOnlyRobotPose().getTranslation()));

        Logger.recordOutput("Robot/Timing/Active", RobotState.isHubActive());
        Logger.recordOutput("Robot/Timing/Shift Time", Math.ceil(RobotState.timeUntilShiftChange()));

        logField.setRobotPose(RobotState.get().getRobotPose());
        logField.getObject("Target").setPose(new Pose2d(ShootCalculator.getShootParams().virtualTarget(), Rotation2d.kZero));
        SmartDashboard.putData("Field", logField);

        if(GeneralConstants.kRobotMode.isSim()) {
            Simulation.periodic();
        }

        timebar.update();
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void reset() {
        if (GeneralConstants.kRobotMode.isSim())
            Simulation.reset();

        RobotState.setShootingMode(ShootingMode.ON_MOVE);

        if (DriverStation.isTeleop()) {
            if (GeneralConstants.kRobotMode.isComp()) {
                shootMachine.forceState(ShootMachine.ShootState.IDLE);
                shootMachine.changeStateForce(ShootMachine.ShootState.RESET);

                intake.forceState(Intake.IntakeStates.IDLE);
                intake.changeStateForce(Intake.IntakeStates.RESET);

                intakeRail.forceState(IntakeRail.IntakeRailState.CLOSED);
                intakeRail.changeStateForce(IntakeRail.IntakeRailState.OPENED);

                box.forceState(Box.BoxState.CLOSED);

                swerveSubsystem.reset();
                Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
                Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);
            } else {
                resetStatemachines();

                Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
                Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);
            }
        } else {
            shootMachine.forceState(ShootMachine.ShootState.IDLE);
            shootMachine.changeStateForce(ShootMachine.ShootState.RESET);

            intake.forceState(Intake.IntakeStates.IDLE);
            intake.changeStateForce(Intake.IntakeStates.RESET);

            intakeRail.forceState(IntakeRail.IntakeRailState.CLOSED);
            intakeRail.changeStateForce(IntakeRail.IntakeRailState.OPENED);

            box.forceState(Box.BoxState.CLOSED);

            swerveSubsystem.reset();
            Swerve.getInstance().setMaxSkidAcceleration(Double.MAX_VALUE);
            Swerve.getInstance().setMaxForwardAcceleration(Double.MAX_VALUE);
        }
    }

    public static void resetStatemachines() {
        shootMachine.forceState(ShootMachine.ShootState.IDLE);
        shootMachine.changeStateForce(ShootMachine.ShootState.RESET);

        intake.forceState(Intake.IntakeStates.IDLE);
        intake.changeStateForce(Intake.IntakeStates.RESET);

        intakeRail.forceState(IntakeRail.IntakeRailState.UNKNOWN);
        intakeRail.changeStateForce(IntakeRail.IntakeRailState.RESET);

        box.forceState(Box.BoxState.UNKNOWN);
        box.changeStateForce(Box.BoxState.RESET);

        swerveSubsystem.reset();
    }
}
