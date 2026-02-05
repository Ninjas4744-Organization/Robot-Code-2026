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
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.accelerator.Accelerator;
import frc.robot.subsystems.accelerator.AcceleratorIO;
import frc.robot.subsystems.accelerator.AcceleratorIOController;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOController;
import frc.robot.subsystems.climberangle.ClimberAngle;
import frc.robot.subsystems.climberangle.ClimberAngleIO;
import frc.robot.subsystems.climberangle.ClimberAngleIOController;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOController;
import frc.robot.subsystems.indexer2.Indexer2;
import frc.robot.subsystems.indexer2.Indexer2IO;
import frc.robot.subsystems.indexer2.Indexer2IOController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOController;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.intakeangle.IntakeAngleIO;
import frc.robot.subsystems.intakeangle.IntakeAngleIOController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOController;
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
    private static IntakeAngle intakeAngle;
    private static Indexer indexer;
    private static Indexer2 indexer2;
    private static Shooter shooter;
    private static Accelerator accelerator;
    private static Climber climber;
    private static ClimberAngle climberAngle;

    private LoggedCommandController driverController;
    private LoggedDashboardChooser<Command> autoChooser;
    List<GamePieceProjectile> balls = new ArrayList<>();
    private List<Translation2d> lastRobotVels = new  ArrayList<>();
    private static Translation2d robotAcc = new Translation2d();
    private final int robotVelsCount = 5;

    public RobotContainer() {
        switch (GeneralConstants.kRobotMode) {
            case WORKSHOP, COMP, SIM, SIM_COMP:
                intake = new Intake(true, new IntakeIOController());
                intakeAngle = new IntakeAngle(false, new IntakeAngleIOController());
                indexer = new Indexer(true, new IndexerIOController());
                indexer2 =  new Indexer2(true, new Indexer2IOController());
                shooter = new Shooter(true, new ShooterIOController());
                accelerator =  new Accelerator(true, new AcceleratorIOController());
                climber = new Climber(false, new ClimberIOController());
                climberAngle = new ClimberAngle(false, new ClimberAngleIOController());

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
                break;

            case REPLAY, REPLAY_COMP:
                intake = new Intake(true, new IntakeIO() {});
                intakeAngle = new IntakeAngle(false, new IntakeAngleIO() {});
                indexer = new Indexer(true, new IndexerIO() {});
                indexer2 =  new Indexer2(true, new Indexer2IO() {});
                shooter = new Shooter(true, new ShooterIO() {});
                accelerator =  new Accelerator(true, new AcceleratorIO() {});
                climber = new Climber(false, new ClimberIO() {});
                climberAngle = new ClimberAngle(false, new ClimberAngleIO() {});

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
                break;
        }

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
                    if (balls.size() >= 8) {
                        SimulatedArena.getInstance().removeProjectile(balls.get(0));
                        balls.remove(0);
                    }

                    GamePieceProjectile ball = new RebuiltFuelOnFly(
                        RobotState.getInstance().getRobotPose().getTranslation(),
                        new Translation2d(),
                        Swerve.getInstance().getSpeeds().getAsFieldRelative(RobotState.getInstance().getRobotPose().getRotation()),
                        RobotState.getInstance().getRobotPose().getRotation(),
                        Meters.of(0.25),
                        MetersPerSecond.of(13.35 * Math.abs(shooter.getGoal()) / 100),
                        Degrees.of(58)
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

    public static IntakeAngle getIntakeAngle() {
        return intakeAngle;
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

        NamedCommands.registerCommand("Prepare Shoot", Commands.sequence(
            swerveSubsystem.lookHub(),
            shooter.autoHubVelocity()
        ));

        NamedCommands.registerCommand("Shoot", new DetachedCommand(Commands.sequence(
            Commands.waitUntil(() -> shooter.atGoal() && swerveSubsystem.atGoal()),
            indexer.setPercent(0.3),
            indexer2.setPercent(0.3),
            accelerator.setVelocity(80)
        )));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
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

    private void configureBindings() {
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(visionSubsystem.getLastMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.povRight().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.RESET, true, false)));
//        driverController.povUp().onTrue(Commands.runOnce(() -> ));

        driverController.L1().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.IDLE, true, false)));

        driverController.R1().onTrue(notTest(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.INTAKE_WHILE_DELIVERY_READY);
            StateMachine.getInstance().changeRobotState(States.INTAKE);
        })));

        driverController.R2().onTrue(notTest(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.SHOOT);
            StateMachine.getInstance().changeRobotState(States.SHOOT_READY);
        })));

        driverController.L2().onTrue(notTest(Commands.runOnce(() -> {
            // Climbing shit
        })));

        driverController.square().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.DUMP)));
    }

    private void configureTestBindings() {
        driverController.R1().toggleOnTrue(inTest(Commands.startEnd(
            () -> CommandScheduler.getInstance().schedule(intake.setPercent(0.35)),
            () -> CommandScheduler.getInstance().schedule(intake.stop())
        )));

        driverController.R2().toggleOnTrue(inTest(Commands.startEnd(
            () -> {
                CommandScheduler.getInstance().schedule(Commands.sequence(
                    Commands.runOnce(() -> {
                        SubsystemConstants.kSwerve.limits.maxSkidAcceleration = 12.5;
                        GeneralConstants.Swerve.kDriverSpeedFactor = 0.3;
                    }),
                    swerveSubsystem.lookHub(),
                    shooter.autoHubVelocity(),
                    accelerator.setVelocity(80),
                    Commands.run(() -> {
                        if (shooter.atGoal() && swerveSubsystem.atGoal() && accelerator.atGoal()) {
                            CommandScheduler.getInstance().schedule(Commands.sequence(
                                indexer.setPercent(0),
                                indexer2.setPercent(0.3)
                            ));
                        } else {
                            CommandScheduler.getInstance().schedule(Commands.sequence(
                                indexer.setPercent(0),
                                indexer2.setPercent(-0.1)
                            ));
                        }
                    })
                ));
            },
            () -> {
                CommandScheduler.getInstance().schedule(Commands.sequence(
                    Commands.runOnce(() -> {
                        SubsystemConstants.kSwerve.limits.maxSkidAcceleration = 80;
                        GeneralConstants.Swerve.kDriverSpeedFactor = 1;
                    }),
                    swerveSubsystem.stop(),
                    indexer.stop(),
                    indexer2.stop(),
                    accelerator.stop(),
                    shooter.stop()
                ));
            }
        )));

        driverController.L1().toggleOnTrue(inTest(Commands.startEnd(() -> visionSubsystem.setEnabled(false), () -> visionSubsystem.setEnabled(true))));
        driverController.L2().onTrue(inTest(Commands.runOnce(() -> RobotState.getInstance().setOdometryOnlyRobotPose(visionSubsystem.getLastVisionPose()))));
    }

    public static Translation2d getRobotAcceleration() {
        return robotAcc;
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        Translation2d robotVel = Swerve.getInstance().getSpeeds().getAsFieldRelative(RobotState.getInstance().getRobotPose().getRotation()).toTranslation();
        lastRobotVels.add(robotVel);
        if (lastRobotVels.size() > robotVelsCount)
            lastRobotVels.remove(0);
        if (lastRobotVels.size() >= 2) {
            // Linear Regression Variables
            double sumT = 0;
            Translation2d sumP = new Translation2d();
            Translation2d sumTP = new Translation2d();
            double sumT2 = 0;
            int n = lastRobotVels.size();

            for (int i = 0; i < n; i++) {
                sumT += i * 0.02;
                sumP = sumP.plus(lastRobotVels.get(i));
                sumTP = sumTP.plus(lastRobotVels.get(i).times(i * 0.02));
                sumT2 += (i * 0.02) * (i * 0.02);
            }

            // Formula for slope (velocity):
            // m = (n*sumTP - sumT*sumP) / (n*sumT2 - sumT^2)
            double denominator = (n * sumT2 - (sumT * sumT));

            // Avoid division by zero if timestamps are identical
            robotAcc = (denominator == 0) ? new Translation2d() : (sumTP.times(n).minus(sumP.times(sumT))).div(denominator);
        }

        Logger.recordOutput("Robot/Distance Hub", FieldConstants.getDistToHub());
        Logger.recordOutput("Robot/Distance Fake Hub", RobotState.getInstance().getDistToHub());
        Logger.recordOutput("Robot/Robot Speed", Swerve.getInstance().getSpeeds().getSpeed());
        Logger.recordOutput("Robot/MegaTag 1 Vision", visionSubsystem.getLastMegaTag1Pose());
        Logger.recordOutput("Robot/Look Ahead Target", new Pose3d(RobotState.getInstance().getHubTargetPose().getX(), RobotState.getInstance().getHubTargetPose().getY(), FieldConstants.getHubPose().getZ(), Rotation3d.kZero));
        Logger.recordOutput("Robot/Shooting Ready", shooter.atGoal() && swerveSubsystem.atGoal() && accelerator.atGoal());
        Logger.recordOutput("Robot/Odometry Only Pose", RobotState.getInstance().getOdometryOnlyRobotPose());
        Logger.recordOutput("Robot/Odometry Vision Error", visionSubsystem.getLastVisionPose().getTranslation().getDistance(RobotState.getInstance().getOdometryOnlyRobotPose().getTranslation()));

        if(GeneralConstants.kRobotMode.isSim()) {
            SimulatedArena.getInstance().simulationPeriodic();

            Pose3d[] balls = new Pose3d[this.balls.size()];
            for (int i = 0; i < this.balls.size(); i++) {
                balls[i] = this.balls.get(i).getPose3d();
            }
            Logger.recordOutput("Robot/Balls", balls);
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
