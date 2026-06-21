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
import frc.lib.NinjasLib.commands.DetachedCommand;
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
    private static Indexer indexer;
    private static Shooter shooter;
    private static Accelerator accelerator;
    private static ShootMachine shootMachine;

    private LoggedCommandController driverController;
    private LoggedCommandController operatorController;
    private LoggedDashboardChooser<Command> autoChooser;
    private ShootCalculator shootCalculator;
    private NinjasTimebar timebar;
    private Field2d logField = new Field2d();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        if (!GeneralConstants.kRobotMode.isReplay()) {
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
            operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIOPS5(GeneralConstants.kOperatorControllerPort));
        } else {
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
            operatorController = new LoggedCommandController("Operator", new LoggedCommandControllerIO() {});
        }

        swerveSubsystem = new SwerveSubsystem(true, false, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, driverController::getRightY);
        RobotStateBase.set(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        visionSubsystem = new VisionSubsystem();
        shootCalculator = new ShootCalculator();

        intake = new Intake(true);
        intakeRail = new IntakeRail(true);
        indexer = new Indexer(true);
        shooter = new Shooter(true);
        accelerator = new Accelerator(true);

        shootMachine = new ShootMachine();

        configureAuto();
        Triggers.setControllers(driverController, operatorController);
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
    public static Indexer getIndexer() { return indexer; }
    public static Shooter getShooter() { return shooter; }
    public static Accelerator getAccelerator() { return accelerator; }
    public static ShootMachine getShootMachine() { return shootMachine; }

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

        NamedCommands.registerCommand("Shoot", new DetachedCommand(Commands.sequence(
            shootMachine.changeStateCommand(ShootMachine.ShootState.PREPARE_HUB),
            RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.SOFT_PUMPING),
            Commands.waitSeconds(1),
            RobotContainer.getIntakeRail().changeStateCommand(IntakeRail.IntakeRailState.HARD_PUMPING)
        )));

        NamedCommands.registerCommand("Stop", Commands.sequence(
            swerveSubsystem.stopCmd(),
            shootMachine.changeStateCommand(ShootMachine.ShootState.REVERSE_BALLS),
            intakeRail.changeStateForceCommand(IntakeRail.IntakeRailState.OPENED)
        ));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    public void controllerPeriodic() {
        driverController.periodic();
        operatorController.periodic();
    }

    public void periodic() {
        SwerveSpeeds robotVel = Swerve.getInstance().getSpeeds();

        Logger.recordOutput("Robot Speed", robotVel.getSpeed());
        Logger.recordOutput("Robot Acceleration", swerveSubsystem.getAcceleration());

        Logger.recordOutput("Hub Active", RobotState.isHubActive());
        Logger.recordOutput("Shift Time", Math.ceil(RobotState.timeUntilShiftChange()));

        logField.setRobotPose(RobotState.get().getRobotPose());
        logField.getObject("Target").setPose(new Pose2d(ShootCalculator.getShootParams().virtualTarget(), Rotation2d.kZero));
        SmartDashboard.putData("Field", logField);

//        if ((DriverStation.isAutonomousEnabled() && (RobotController.getFPGATime() - Robot.autoStartTime) / 1000000.0 > 19.5)
//            || (DriverStation.isTeleopEnabled() && (RobotController.getFPGATime() - Robot.teleopStartTime) / 1000000.0 > 139.5)) {
//            shootMachine.changeStateForce(ShootMachine.ShootState.IDLE);
//            intake.changeStateForce(Intake.IntakeStates.IDLE);
//        }

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

        RobotState.setShootingMode(ShootingMode.HUB);

        if (DriverStation.isTeleop()) {
            if (GeneralConstants.kRobotMode.isComp()) {
                resetStatemachines(false);
                Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
                Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);
            } else {
                resetStatemachines(true);
                Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
                Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);
            }
        } else {
            resetStatemachines(false);
            Swerve.getInstance().setMaxSkidAcceleration(Double.MAX_VALUE);
            Swerve.getInstance().setMaxForwardAcceleration(Double.MAX_VALUE);
        }
    }

    public static void resetStatemachines(boolean resetIntakeRail) {
        shootMachine.forceState(ShootMachine.ShootState.IDLE);
        shootMachine.changeStateForce(ShootMachine.ShootState.RESET);

        intake.forceState(Intake.IntakeStates.IDLE);
        intake.changeStateForce(Intake.IntakeStates.INTAKE);

        if (resetIntakeRail) {
            intakeRail.forceState(IntakeRail.IntakeRailState.UNKNOWN);
            intakeRail.changeStateForce(IntakeRail.IntakeRailState.RESET);
        } else {
            intakeRail.forceState(IntakeRail.IntakeRailState.CLOSED);
            intakeRail.changeStateForce(IntakeRail.IntakeRailState.OPENED);
        }

        swerveSubsystem.reset();
    }
}
