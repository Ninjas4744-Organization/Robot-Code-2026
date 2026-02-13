package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.*;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.*;

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

    private LoggedCommandController driverController;
    private LoggedDashboardChooser<Command> autoChooser;

    List<GamePieceProjectile> balls = new ArrayList<>();
    private static DerivativeCalculator2d accelerationCalculator = new DerivativeCalculator2d(5);

    public RobotContainer() {
        intake = new Intake(true);
        intakeOpen = new IntakeOpen(true);
        indexer = new Indexer(true);
        indexer2 = new Indexer2(true);
        shooter = new Shooter(true);
        accelerator = new Accelerator(true);
        climber = new Climber(true);
        climberAngle = new ClimberAngle(true);

        if (!GeneralConstants.kRobotMode.isReplay())
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
        else
            driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});

        swerveSubsystem = new SwerveSubsystem(true, false, driverController::getLeftX, driverController::getLeftY, driverController::getRightX, driverController::getRightY);
        RobotStateBase.setInstance(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        StateMachineBase.setInstance(new StateMachine());
        visionSubsystem = new VisionSubsystem();

        configureAuto();
        configureBindings();
        configureTestBindings();

        if (GeneralConstants.kRobotMode.isSim()) {
            CommandScheduler.getInstance().schedule(Commands.runOnce(() -> {
                if (Math.abs(shooter.getVelocity()) > 0.01) {
                    if (balls.size() >= 15) {
                        SimulatedArena.getInstance().removeProjectile(balls.get(0));
                        balls.remove(0);
                    }

                    GamePieceProjectile ball = new RebuiltFuelOnFly(
                        RobotState.getInstance().getRobotPose().getTranslation(),
                        new Translation2d(),
                        Swerve.getInstance().getSpeeds().getAsFieldRelative(RobotState.getInstance().getRobotPose().getRotation()),
                        RobotState.getInstance().getRobotPose().getRotation(),
                        Meters.of(0.481),
                        MetersPerSecond.of(13.25 * Math.abs(shooter.getGoal()) / 100),
                        Degrees.of(60)
                    );

                    balls.add(ball);
                    SimulatedArena.getInstance().addGamePieceProjectile(ball);
                }
            }).andThen(Commands.waitSeconds(0.1)).repeatedly().ignoringDisable(true));
        }
    }

    public static SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public static VisionSubsystem getVision() {
        return visionSubsystem;
    }

    public static Intake getIntake() {
        return intake;
    }

    public static IntakeOpen getIntakeAngle() {
        return intakeOpen;
    }

    public static Indexer getIndexer() {
        return indexer;
    }

    public static Indexer2 getIndexer2() {
        return indexer2;
    }

    public static Shooter getShooter() {
        return shooter;
    }

    public static Accelerator getAccelerator() {
        return accelerator;
    }

    public static Climber getClimber() {
        return climber;
    }

    public static ClimberAngle getClimberAngle() {
        return climberAngle;
    }

    private void configureAuto() {
        AutoBuilder.configure(
            RobotState.getInstance()::getRobotPose,
            RobotState.getInstance()::setRobotPose,
            Swerve.getInstance()::getSpeeds,
            swerveSubsystem::setAutoInput,
            SubsystemConstants.kAutonomyConfig,
            SubsystemConstants.kSwerve.special.robotConfig,
            () -> false
        );

        NamedCommands.registerCommand("Prepare Shoot", StateMachine.getInstance().changeRobotStateCommand(States.SHOOT_READY));
        NamedCommands.registerCommand("Shoot", StateMachine.getInstance().changeRobotStateCommand(States.SHOOT));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(visionSubsystem.getLastMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.povRight().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.RESET, true, false)));
        driverController.povUp().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.DUMP)));

        driverController.L1().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.IDLE, true, false)));

        driverController.R1().onTrue(notTest(intake()));

        driverController.R2().onTrue(notTest(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.SHOOT);
            StateMachine.getInstance().changeRobotState(States.SHOOT_READY);
        })));

        driverController.cross().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.ON_MOVE)));
        driverController.circle().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.SNAP_RING)));
        driverController.triangle().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.LOCK)));
        driverController.square().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.DELIVERY)));

        driverController.L2().onTrue(notTest(Commands.runOnce(() -> {
            // Climbing shit
        })));
    }

    private Command intake() {
        return Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(switch (RobotState.getInstance().getRobotState()) {
                case IDLE -> States.INTAKE;
                case SHOOT_HEATED -> States.INTAKE_WHILE_SHOOT_HEATED;
                case SHOOT_READY -> States.INTAKE_WHILE_SHOOT_READY;
                case SHOOT -> States.SHOOT;
                default -> RobotState.getInstance().getRobotState();
            });
        });
    }

    private Command inTest(Command command) {
        return Commands.either(
            command,
            Commands.none(),
            DriverStation::isTest
        );
    }

    private Command notTest(Command command) {
        return Commands.either(
            Commands.none(),
            command,
            DriverStation::isTest
        );
    }

    private void configureTestBindings() {
//        driverController.R1().toggleOnTrue(inTest(Commands.startEnd(
//            () -> intake.setVelocity(0.35),
//            () -> intake.stop()
//        )));
//
//        driverController.R2().toggleOnTrue(inTest(Commands.startEnd(
//            () -> CommandScheduler.getInstance().schedule(Commands.sequence(
//                Commands.runOnce(() -> swerveSubsystem.lookHub()),
//                Commands.runOnce(() -> shooter.autoHubVelocity()),
//                Commands.waitUntil(() -> shooter.atGoal() && swerveSubsystem.atGoal()),
//                indexer.setPercentCmd(0.3),
//                indexer2.setPercentCmd(0.3),
//                accelerator.setVelocityCmd(80)
//            )),
//            () ->
//                CommandScheduler.getInstance().schedule(Commands.sequence(
//                    Commands.runOnce(() -> {
//                        SubsystemConstants.kSwerve.limits.maxSkidAcceleration = 12.5;
//                        GeneralConstants.Swerve.kDriverSpeedFactor = 0.3;
//                    }),
//                    Commands.runOnce(() -> swerveSubsystem.lookHub()),
//                    Commands.runOnce(() -> shooter.autoHubVelocity()),
//                    Commands.runOnce(() -> accelerator.setVelocity(80)),
//                    Commands.run(() -> {
//                        if (shooter.atGoal() && swerveSubsystem.atGoal() && accelerator.atGoal()) {
//                            CommandScheduler.getInstance().schedule(Commands.sequence(
//                                indexer.setPercentCmd(0),
//                                indexer2.setPercentCmd(0.3)
//                            ));
//                        } else {
//                            CommandScheduler.getInstance().schedule(Commands.sequence(
//                                indexer.setPercentCmd(0),
//                                indexer2.setPercentCmd(-0.1)
//                            ));
//                        }
//                    })
//                ));,
//            () -> {
//                CommandScheduler.getInstance().schedule(Commands.sequence(
//                    Commands.runOnce(() -> {
//                        SubsystemConstants.kSwerve.limits.maxSkidAcceleration = 80;
//                        GeneralConstants.Swerve.kDriverSpeedFactor = 1;
//                    }),
//                    swerveSubsystem.stop(),
//                    indexer.stopCmd(),
//                    indexer2.stopCmd(),
//                    accelerator.stopCmd(),
//                    shooter.stopCmd()
//                ));
//            }
//        )));

        driverController.L1().toggleOnTrue(inTest(Commands.startEnd(() -> visionSubsystem.setEnabled(false), () -> visionSubsystem.setEnabled(true))));
        driverController.L2().onTrue(inTest(Commands.runOnce(() -> RobotState.getInstance().setOdometryOnlyRobotPose(visionSubsystem.getLastVisionPose()))));
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
        accelerationCalculator.calculate(robotVel.getAsFieldRelative(RobotState.getInstance().getRobotPose().getRotation()).toTranslation());

        Logger.recordOutput("Robot/Shooting/Distance Hub", FieldConstants.getDistToHub());
        Logger.recordOutput("Robot/Shooting/Distance Lookahead Hub", RobotState.getInstance().getLookaheadTargetDist());
        Logger.recordOutput("Robot/Shooting/Lookahead Target", new Pose3d(RobotState.getInstance().getLookaheadTargetPose().getX(), RobotState.getInstance().getLookaheadTargetPose().getY(), FieldConstants.getHubPose().getZ(), Rotation3d.kZero));
        Logger.recordOutput("Robot/Shooting/Shooting Ready", RobotState.isShootReady());

        Logger.recordOutput("Robot/Robot Speed", robotVel.getSpeed());
        Logger.recordOutput("Robot/Robot Acceleration 2d", getRobotAcceleration());
        Logger.recordOutput("Robot/Robot Acceleration", getRobotAcceleration().getNorm());
        Logger.recordOutput("Robot/Robot Predicted Velocity", new SwerveSpeeds(robotVel.getAsFieldRelative().toTranslation().plus(getRobotAcceleration().times(0.2)), robotVel.omegaRadiansPerSecond, true));

        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision", visionSubsystem.getLastMegaTag1Pose());
        Logger.recordOutput("Robot/Vision/Odometry Only Pose", RobotState.getInstance().getOdometryOnlyRobotPose());
        Logger.recordOutput("Robot/Vision/Odometry Vision Error", visionSubsystem.getLastVisionPose().getTranslation().getDistance(RobotState.getInstance().getOdometryOnlyRobotPose().getTranslation()));

        if (RobotState.getInstance().getDistance(visionSubsystem.getLastMegaTag1Pose()) <= 1)
            RobotState.getInstance().resetGyro(Swerve.getInstance().getGyro().getYaw().times(0.9).plus(visionSubsystem.getLastMegaTag1Pose().getRotation().times(0.1)));

        if(GeneralConstants.kRobotMode.isSim()) {
            SimulatedArena.getInstance().simulationPeriodic();

            Pose3d[] balls = new Pose3d[this.balls.size()];
            for (int i = 0; i < this.balls.size(); i++) {
                balls[i] = this.balls.get(i).getPose3d();
            }
            Logger.recordOutput("Robot/Shooting/Balls", balls);
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    public void reset() {
//        RobotState.getInstance().resetGyro(visionSubsystem.getLastMegaTag1Pose().getRotation());
        if (GeneralConstants.kRobotMode.isComp()) {
            StateMachine.getInstance().changeRobotState(States.STARTING_POSE, false, true);
            StateMachine.getInstance().changeRobotState(States.IDLE, true, false);
        }
        else {
            StateMachine.getInstance().changeRobotState(States.UNKNOWN, false, true);
            StateMachine.getInstance().changeRobotState(States.RESET, true, false);
        }
    }
}
