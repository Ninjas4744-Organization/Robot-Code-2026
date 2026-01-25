package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
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
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private LoggedCommandController driverController;
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

        configureBindings();
        configureTestBindings();
    }

    public static SwerveSubsystem getSwerve() {
        return swerveSubsystem;
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

        driverController.L1().onTrue(notTest(StateMachine.getInstance().changeRobotStateCommand(States.IDLE)));

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
            () -> CommandScheduler.getInstance().schedule(intake.setPercent(0.5)),
            () -> CommandScheduler.getInstance().schedule(intake.stop())
        )));

        driverController.R2().toggleOnTrue(inTest(Commands.startEnd(
            () -> {
                CommandScheduler.getInstance().schedule(swerveSubsystem.lookHub());
                CommandScheduler.getInstance().schedule(indexer.setPercent(0.3));
                CommandScheduler.getInstance().schedule(indexer2.setPercent(0.3));
                CommandScheduler.getInstance().schedule(accelerator.setVelocity(80));
//                CommandScheduler.getInstance().schedule(shooter.setVelocity(50));
                CommandScheduler.getInstance().schedule(shooter.autoHubVelocity());
            },
            () -> {
                CommandScheduler.getInstance().schedule(swerveSubsystem.stop());
                CommandScheduler.getInstance().schedule(indexer.stop());
                CommandScheduler.getInstance().schedule(indexer2.stop());
                CommandScheduler.getInstance().schedule(accelerator.stop());
                CommandScheduler.getInstance().schedule(shooter.stop());
            }
        )));
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
        Logger.recordOutput("Distance", FieldConstants.getDistToHub());

        if(GeneralConstants.kRobotMode.isSim())
            SimulatedArena.getInstance().simulationPeriodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void reset() {
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
