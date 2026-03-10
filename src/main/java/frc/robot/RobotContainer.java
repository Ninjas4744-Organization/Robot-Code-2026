package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
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
    private static IntakeOpen intakeOpen;
    private static Box box;
    private static Indexer indexer;
    private static Shooter shooter;
    private static Accelerator accelerator;
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
        RobotStateBase.setInstance(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        visionSubsystem = new VisionSubsystem();
        shootCalculator = new ShootCalculator();

        intake = new Intake(true);
        intakeOpen = new IntakeOpen(true);
        box = new Box(false);
        indexer = new Indexer(true);
        shooter = new Shooter(true);
        accelerator = new Accelerator(true);
        leds = new Leds(false);

        StateMachineBase.setInstance(new StateMachine());

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
    public static IntakeOpen getIntakeOpen() { return intakeOpen; }
    public static Box getBox() { return box; }
    public static Indexer getIndexer() { return indexer; }
    public static Shooter getShooter() { return shooter; }
    public static Accelerator getAccelerator() { return accelerator; }
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
            RobotState.setAutoSwitchShootReadyToShoot(true);
            StateMachine.getInstance().changeRobotState(States.SHOOT_PREPARE);
        }));

        NamedCommands.registerCommand("Stop", Commands.runOnce(() -> {
            RobotState.setAutoSwitchShootReadyToShoot(false);
            StateMachine.getInstance().changeRobotState(States.IDLE);
        }));

        NamedCommands.registerCommand("Intake", Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.INTAKE_BOX_OPENED);
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

        if (RobotState.isTeleop()) {
            Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
            Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);

            if (GeneralConstants.kRobotMode.isComp()) {
                StateMachine.getInstance().changeRobotStateForce(States.INTAKE_BOX_CLOSED);
            } else {
                StateMachine.getInstance().forceRobotState(States.UNKNOWN);
                StateMachine.getInstance().changeRobotStateForce(States.RESET);
            }
        } else {
            Swerve.getInstance().setMaxSkidAcceleration(Double.MAX_VALUE);
            Swerve.getInstance().setMaxForwardAcceleration(Double.MAX_VALUE);

            StateMachine.getInstance().forceRobotState(States.STARTING_POSE);
            StateMachine.getInstance().changeRobotStateForce(States.INTAKE_BOX_CLOSED);
        }
    }
}
