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
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {
    private LoggedCommandController driverController;
    private static SwerveSubsystem swerveSubsystem;
    private static Intake intake;
    private static IntakeAngle intakeAngle;
    private static IntakeIndexer intakeIndexer;
    private static Shooter shooter;
    private static ShooterIndexer shooterIndexer;

    public RobotContainer() {
        switch (GeneralConstants.kRobotMode) {
            case WORKSHOP, COMP, SIM, SIM_COMP:
                intake = new Intake(false, new IntakeIOController());
                intakeAngle = new IntakeAngle(false, new IntakeAngleIOController());
                intakeIndexer = new IntakeIndexer(false, new IntakeIndexerIOController());
                shooter = new Shooter(false, new ShooterIOController());
                shooterIndexer =  new ShooterIndexer(false, new ShooterIndexerIOController());

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(GeneralConstants.kDriverControllerPort));
                break;

            case REPLAY, REPLAY_COMP:
                intake = new Intake(false, new IntakeIO() {});
                intakeAngle = new IntakeAngle(false, new IntakeAngleIO() {});
                intakeIndexer = new IntakeIndexer(false, new IntakeIndexerIO() {});
                shooter = new Shooter(false, new ShooterIO() {});
                shooterIndexer =  new ShooterIndexer(false, new ShooterIndexerIO() {});

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
                break;
        }

        swerveSubsystem = new SwerveSubsystem(true);
        RobotStateBase.setInstance(new RobotState(SubsystemConstants.kSwerve.chassis.kinematics));
        StateMachineBase.setInstance(new StateMachine());
        new VisionSubsystem();

        configureBindings();
    }

    public static SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    static Intake getIntake() {
        return intake;
    }

    static IntakeAngle getIntakeAngle() {
        return intakeAngle;
    }

    static IntakeIndexer getIntakeIndexer() {
        return intakeIndexer;
    }

    static Shooter getShooter() {
        return shooter;
    }

    static ShooterIndexer getShooterIndexer() {
        return shooterIndexer;
    }

    private void configureBindings() {
        driverController.povUp().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(RobotState.getInstance().getRobotPose().getRotation())));
        driverController.povRight().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.RESET, true)));
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
            RobotState.getInstance().setRobotState(States.STARTING_POSE);
            StateMachine.getInstance().changeRobotState(States.IDLE, true);
        }
        else {
            RobotState.getInstance().setRobotState(States.UNKNOWN);
            StateMachine.getInstance().changeRobotState(States.RESET, true);
        }
    }
}
