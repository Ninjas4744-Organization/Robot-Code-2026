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
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import java.util.Set;

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
    private static Leds leds;

    private LoggedCommandController driverController;
    private LoggedDashboardChooser<Command> autoChooser;

    List<GamePieceProjectile> simBalls = new ArrayList<>();
    private static DerivativeCalculator2d accelerationCalculator = new DerivativeCalculator2d(5);

    public RobotContainer() {
        intake = new Intake(false);
        intakeOpen = new IntakeOpen(false);
        indexer = new Indexer(false);
        indexer2 = new Indexer2(false);
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
        configureBindings();
        configureTestBindings();

        new Trigger(RobotState::isHubActive)
            .onChange(Commands.runOnce(() -> {
                if (!GeneralConstants.enableAutoTiming)
                    return;

                if (RobotState.isHubActive()) RobotState.setShootingMode(States.ShootingMode.ON_MOVE);
                else                          RobotState.setShootingMode(States.ShootingMode.DELIVERY);

                if (Set.of(States.SHOOT_HEATED,
                    States.SHOOT_PREPARE,
                    States.SHOOT_READY,
                    States.SHOOT)
                    .contains(RobotState.getInstance().getRobotState()))
                    StateMachine.getInstance().changeRobotStateForce(States.IDLE);
            }));

        new Trigger(() -> RobotState.isTeleop() && (swerveSubsystem.nearRightTrench() || swerveSubsystem.nearLeftTrench()))
            .onTrue(Commands.runOnce(swerveSubsystem::autoTrench))
            .onFalse(Commands.runOnce(swerveSubsystem::stop));

        if (GeneralConstants.kRobotMode.isSim()) {
            CommandScheduler.getInstance().schedule(Commands.runOnce(() -> {
                if (Math.abs(shooter.getVelocity()) > 1 && RobotState.getInstance().getRobotState() == States.SHOOT) {
                    if (simBalls.size() >= 15) {
                        SimulatedArena.getInstance().removeProjectile(simBalls.get(0));
                        simBalls.remove(0);
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

                    simBalls.add(ball);
                    SimulatedArena.getInstance().addGamePieceProjectile(ball);
                }
            }).andThen(Commands.waitSeconds(0.1)).repeatedly().ignoringDisable(true));
        }
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
            RobotState.getInstance()::getRobotPose,
            RobotState.getInstance()::setRobotPose,
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

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
    }

    private void configureBindings() {
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(visionSubsystem.getMegaTag1Pose() == null ? Rotation2d.kZero : visionSubsystem.getMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.povRight().onTrue(notTest(StateMachine.getInstance().changeRobotStateForceCommand(States.RESET)));
        driverController.povUp().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.DUMP)));

        driverController.L1().onTrue(notTest(Commands.runOnce(() -> {
            RobotState.setAutoReadyToShoot(false);
            swerveSubsystem.stop();
            RobotState.setIntake(false);
            StateMachine.getInstance().changeRobotStateForce(States.IDLE);
        })));

        driverController.R1().onTrue(notTest(Commands.runOnce(() -> RobotState.setIntake(true))));

        driverController.R2().onTrue(notTest(Commands.runOnce(() -> {
            if (RobotState.getInstance().getRobotState() == States.SHOOT) {
                RobotState.setAutoReadyToShoot(false);
                StateMachine.getInstance().changeRobotState(States.SHOOT_READY);
            } else {
                RobotState.setAutoReadyToShoot(true);
                StateMachine.getInstance().changeRobotState(States.SHOOT_PREPARE);
            }
        })));

        driverController.R3().onTrue(notTest(Commands.runOnce(swerveSubsystem::snapAngle)
            .andThen(Commands.waitUntil(swerveSubsystem::atAngle))
            .finallyDo(swerveSubsystem::stop)));

        driverController.cross().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.ON_MOVE)));
        driverController.circle().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.SNAP_RING)));
        driverController.triangle().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.LOCK)));
        driverController.square().onTrue(Commands.runOnce(() -> RobotState.setShootingMode(States.ShootingMode.DELIVERY)));

        driverController.L2().onTrue(notTest(Commands.runOnce(() -> {
            // Climbing shit
        })));
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
//        driverController.R2().toggleOnTrue(inTest(Commands.startEnd(
//            () -> CommandScheduler.getInstance().schedule(Commands.sequence(
//                Commands.runOnce(() -> shooter.autoHubVelocity()),
//                Commands.waitUntil(() -> shooter.atGoal()),
//                accelerator.setVelocityCmd(80)
//            )),
//            () -> CommandScheduler.getInstance().schedule(Commands.sequence(
//                accelerator.stopCmd(),
//                shooter.stopCmd()
//            ))
//        )));

        driverController.L2().toggleOnTrue(inTest(Commands.startEnd(() -> visionSubsystem.setEnabled(false), () -> visionSubsystem.setEnabled(true))));
        driverController.R2().onTrue(inTest(Commands.runOnce(() -> RobotState.getInstance().setOdometryOnlyRobotPose(visionSubsystem.getLastVisionPose()))));
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

        Logger.recordOutput("Robot/Shooting/Lookahead Target", new Pose3d(RobotState.getInstance().getLookaheadTargetPose(FieldConstants.getHubPose()).getX(), RobotState.getInstance().getLookaheadTargetPose(FieldConstants.getHubPose()).getY(), FieldConstants.getHubPose().getZ(), Rotation3d.kZero));
        Logger.recordOutput("Robot/Shooting/Shooting Ready", RobotState.isShootReady());

        Logger.recordOutput("Robot/Speed/Robot Speed", robotVel.getSpeed());
        Logger.recordOutput("Robot/Speed/Robot Acceleration", getRobotAcceleration());

        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision", visionSubsystem.getMegaTag1Pose());
        Logger.recordOutput("Robot/Vision/MegaTag 1 Vision Dist", visionSubsystem.getMegaTag1DistFromTag());
        Logger.recordOutput("Robot/Vision/Odometry Only Pose", RobotState.getInstance().getOdometryOnlyRobotPose());
        Logger.recordOutput("Robot/Vision/Odometry Vision Error", visionSubsystem.getLastVisionPose().getTranslation().getDistance(RobotState.getInstance().getOdometryOnlyRobotPose().getTranslation()));

        Logger.recordOutput("Robot/Hub/Distance Hub", FieldConstants.getDistToHub());
        Logger.recordOutput("Robot/Hub/Distance Lookahead Hub", RobotState.getInstance().getLookaheadTargetDist(FieldConstants.getHubPose()));
        Logger.recordOutput("Robot/Hub/Active", RobotState.isHubActive());
        Logger.recordOutput("Robot/Hub/Time Until Hub Change", RobotState.timeUntilHubChange());

//        if (visionSubsystem.getMegaTag1Pose() != null && visionSubsystem.getMegaTag1DistFromTag() <= 2)
//            RobotState.getInstance().resetGyro(Swerve.getInstance().getGyro().getYaw().times(0.9).plus(visionSubsystem.getMegaTag1Pose().getRotation().times(0.1)));

        if(GeneralConstants.kRobotMode.isSim()) {
            SimulatedArena.getInstance().simulationPeriodic();

            Pose3d[] balls = new Pose3d[this.simBalls.size()];
            for (int i = 0; i < this.simBalls.size(); i++) {
                balls[i] = this.simBalls.get(i).getPose3d();
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
            StateMachine.getInstance().forceRobotState(States.STARTING_POSE);
            StateMachine.getInstance().changeRobotStateForce(States.IDLE);
        }
        else {
            StateMachine.getInstance().forceRobotState(States.UNKNOWN);
            StateMachine.getInstance().changeRobotStateForce(States.RESET);
        }
    }
}
