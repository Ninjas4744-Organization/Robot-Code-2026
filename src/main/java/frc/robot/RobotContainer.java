package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.DerivativeCalculator2d;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private static SwerveSubsystem swerveSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static Intake intake;
    private static IntakeOpen intakeOpen;
    private static Indexer indexer;
    private static Indexer2 indexer2;
    private static Shooter shooter;
    private static Accelerator accelerator;
    private static Climber climber;
    private static ClimberAngle climberAngle;
    private static Leds leds;

    private LoggedCommandController driverController;
    private LoggedDashboardChooser<Command> autoChooser;
    private static DerivativeCalculator2d accelerationCalculator = new DerivativeCalculator2d(1);
    private NinjasTimebar timebar;
    private Field2d logField = new Field2d();

    public RobotContainer() {
        intake = new Intake(true);
        intakeOpen = new IntakeOpen(false);
        indexer = new Indexer(true);
        indexer2 = new Indexer2(true);
        shooter = new Shooter(true);
        accelerator = new Accelerator(true);
        climber = new Climber(false);
        climberAngle = new ClimberAngle(false);
        leds = new Leds(false);

        if (!GeneralConstants.kRobotMode.isReplay())
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
        else
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});

        swerveSubsystem = new SwerveSubsystem(true, false, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, driverController::getRightY);
        RobotStateBase.setInstance(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        StateMachineBase.setInstance(new StateMachine());
        visionSubsystem = new VisionSubsystem();

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
    public static IntakeOpen getIntakeAngle() { return intakeOpen; }
    public static Indexer getIndexer() { return indexer; }
    public static Indexer2 getIndexer2() { return indexer2; }
    public static Shooter getShooter() { return shooter; }
    public static Accelerator getAccelerator() { return accelerator; }
    public static Climber getClimber() { return climber; }
    public static ClimberAngle getClimberAngle() { return climberAngle; }
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
            RobotState.setAutoReadyToShoot(true);
            StateMachine.getInstance().changeRobotState(States.SHOOT_PREPARE);
        }));

        NamedCommands.registerCommand("Stop", Commands.runOnce(() -> {
            RobotState.setAutoReadyToShoot(false);
            RobotState.setIntake(false);
            StateMachine.getInstance().changeRobotState(States.IDLE);
        }));

        NamedCommands.registerCommand("Intake", Commands.runOnce(() -> {
            RobotState.setIntake(true);
        }));

        NamedCommands.registerCommand("Climb", Commands.sequence(
            StateMachine.getInstance().changeRobotStateCommand(States.IDLE),
            Commands.waitUntil(() -> RobotState.get().getRobotState() == States.IDLE),
            StateMachine.getInstance().changeRobotStateCommand(States.CLIMB1_READY)
        ));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    public static Translation2d getRobotAcceleration() {
        return accelerationCalculator.get();
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        SwerveSpeeds robotVel = Swerve.getInstance().getSpeeds();
        accelerationCalculator.calculate(robotVel.getAsFieldRelative().toTranslation());

        Logger.recordOutput("Robot/Shooting/Lookahead Target", new Pose3d(RobotState.get().getLookaheadTargetPose(FieldConstants.getHubPose()).getX(), RobotState.get().getLookaheadTargetPose(FieldConstants.getHubPose()).getY(), FieldConstants.getHubPose().getZ(), Rotation3d.kZero));
        Logger.recordOutput("Robot/Shooting/Shooting Ready", RobotState.isShootReady());

        Logger.recordOutput("Robot/Speed/Robot Speed", robotVel.getSpeed());
        Logger.recordOutput("Robot/Speed/Robot Acceleration", getRobotAcceleration());

        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision", visionSubsystem.getMegaTag1Pose());
        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision Dist", visionSubsystem.getMegaTag1DistFromTag());
        Logger.recordOutput("Robot/Vision/Odometry Only Pose", RobotState.get().getOdometryOnlyRobotPose());
        Logger.recordOutput("Robot/Vision/Odometry Vision Error", visionSubsystem.getLastVisionPose().getTranslation().getDistance(RobotState.get().getOdometryOnlyRobotPose().getTranslation()));

        Logger.recordOutput("Robot/Hub/Distance Hub", FieldConstants.getDistToHub());
        Logger.recordOutput("Robot/Hub/Distance Lookahead Hub", RobotState.get().getLookaheadTargetDist(FieldConstants.getHubPose()));
        Logger.recordOutput("Robot/Hub/Active", RobotState.isHubActive());
        Logger.recordOutput("Robot/Hub/Time Until Hub Change", RobotState.timeUntilHubChange());

        logField.setRobotPose(RobotState.get().getRobotPose());
        logField.getObject("Target").setPose(RobotState.getShootingMode() == States.ShootingMode.DELIVERY ? PositionsConstants.Swerve.getDeliveryTarget() : RobotState.get().getLookaheadTargetPose(FieldConstants.getHubPose()).toPose2d());
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
        if (RobotState.isTeleop()) {
            Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
            Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);
        } else {
            Swerve.getInstance().setMaxSkidAcceleration(Double.MAX_VALUE);
            Swerve.getInstance().setMaxForwardAcceleration(Double.MAX_VALUE);
        }

        if (GeneralConstants.kRobotMode.isComp()) {
            if (!RobotState.isTeleop()) {
                StateMachine.getInstance().forceRobotState(States.STARTING_POSE);
                StateMachine.getInstance().changeRobotStateForce(States.BALLS_READY);
            } else {
                StateMachine.getInstance().changeRobotState(States.CLIMB_DOWN);
            }
        }
        else {
            StateMachine.getInstance().forceRobotState(States.UNKNOWN);
            StateMachine.getInstance().changeRobotStateForce(States.RESET);
        }
    }
}
