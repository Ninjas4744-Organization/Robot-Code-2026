package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOController;
import frc.robot.subsystems.climberangle.ClimberAngle;
import frc.robot.subsystems.climberangle.ClimberAngleIO;
import frc.robot.subsystems.climberangle.ClimberAngleIOController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOController;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.intakeangle.IntakeAngleIO;
import frc.robot.subsystems.intakeangle.IntakeAngleIOController;
import frc.robot.subsystems.intakeindexer.IntakeIndexer;
import frc.robot.subsystems.intakeindexer.IntakeIndexerIO;
import frc.robot.subsystems.intakeindexer.IntakeIndexerIOController;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOController;
import frc.robot.subsystems.shooterindexer.ShooterIndexer;
import frc.robot.subsystems.shooterindexer.ShooterIndexerIO;
import frc.robot.subsystems.shooterindexer.ShooterIndexerIOController;
import frc.robot.subsystems.shooterindexer2.shooterindexer.ShooterIndexer2;
import frc.robot.subsystems.shooterindexer2.shooterindexer.ShooterIndexer2IO;
import frc.robot.subsystems.shooterindexer2.shooterindexer.ShooterIndexer2IOController;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {
    private LoggedCommandController driverController;
    private static SwerveSubsystem swerveSubsystem;
    private static VisionSubsystem visionSubsystem;
    private static Intake intake;
    private static IntakeAngle intakeAngle;
    private static IntakeIndexer intakeIndexer;
    private static Shooter shooter;
    private static ShooterIndexer shooterIndexer;
    private static ShooterIndexer2 shooterIndexer2;
    private static Climber climber;
    private static ClimberAngle climberAngle;

    public RobotContainer() {
        switch (GeneralConstants.kRobotMode) {
            case WORKSHOP, COMP, SIM, SIM_COMP:
                intake = new Intake(false, new IntakeIOController());
                intakeAngle = new IntakeAngle(false, new IntakeAngleIOController());
                intakeIndexer = new IntakeIndexer(false, new IntakeIndexerIOController());
                shooter = new Shooter(false, new ShooterIOController());
                shooterIndexer =  new ShooterIndexer(false, new ShooterIndexerIOController());
                shooterIndexer2 =  new ShooterIndexer2(false, new ShooterIndexer2IOController());
                climber = new Climber(false, new ClimberIOController());
                climberAngle = new ClimberAngle(false, new ClimberAngleIOController());

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
                break;

            case REPLAY, REPLAY_COMP:
                intake = new Intake(false, new IntakeIO() {});
                intakeAngle = new IntakeAngle(false, new IntakeAngleIO() {});
                intakeIndexer = new IntakeIndexer(false, new IntakeIndexerIO() {});
                shooter = new Shooter(false, new ShooterIO() {});
                shooterIndexer =  new ShooterIndexer(false, new ShooterIndexerIO() {});
                shooterIndexer2 =  new ShooterIndexer2(false, new ShooterIndexer2IO() {});
                climber = new Climber(false, new ClimberIO() {});
                climberAngle = new ClimberAngle(false, new ClimberAngleIO() {});

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
                break;
        }

        swerveSubsystem = new SwerveSubsystem(true, false, () -> driverController.getLeftX(), () -> driverController.getLeftY(), () -> driverController.getRightX(), () -> driverController.getRightY());
        RobotStateBase.setInstance(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        StateMachineBase.setInstance(new StateMachine());
        visionSubsystem = new VisionSubsystem();

        configureBindings();
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

    public static IntakeIndexer getIntakeIndexer() {
        return intakeIndexer;
    }

    public static Shooter getShooter() {
        return shooter;
    }

    public static ShooterIndexer getShooterIndexer() {
        return shooterIndexer;
    }

    public static ShooterIndexer2 getShooterIndexer2() {
        return shooterIndexer2;
    }

    public static Climber getClimber() {
        return climber;
    }

    public static ClimberAngle getClimberAngle() {
        return climberAngle;
    }

    private void configureBindings() {
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(visionSubsystem.getLastMegaTag1Pose().getRotation())));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.povRight().onTrue(StateMachine.getInstance().changeRobotStateCommand(States.RESET, true, false));
//        driverController.povUp().onTrue(Commands.runOnce(() -> ));

        driverController.L1().onTrue(StateMachine.getInstance().changeRobotStateCommand(States.IDLE));

        driverController.R1().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.INTAKE_WHILE_DELIVERY_READY);
            StateMachine.getInstance().changeRobotState(States.INTAKE);
        }));

        driverController.R2().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.SHOOT);
            StateMachine.getInstance().changeRobotState(States.SHOOT_READY);
        }));

        driverController.L2().onTrue(Commands.runOnce(() -> {
            // Climbing shit
        }));

        driverController.square().onTrue(StateMachine.getInstance().changeRobotStateCommand(States.DUMP));
    }

    public void controllerPeriodic() {
        driverController.periodic();
//        operatorController.periodic();
    }

    public void periodic() {
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
