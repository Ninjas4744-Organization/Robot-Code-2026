package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIO;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIOReal;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIOSim;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOController;
import frc.robot.subsystems.intakealigner.IntakeAligner;
import frc.robot.subsystems.intakealigner.IntakeAlignerIO;
import frc.robot.subsystems.intakealigner.IntakeAlignerIOController;
import frc.robot.subsystems.intakeangle.IntakeAngle;
import frc.robot.subsystems.intakeangle.IntakeAngleIO;
import frc.robot.subsystems.intakeangle.IntakeAngleIOController;
import frc.robot.subsystems.outtake.Outtake;
import frc.robot.subsystems.outtake.OuttakeIO;
import frc.robot.subsystems.outtake.OuttakeIOController;
import org.ironmaple.simulation.SimulatedArena;

public class RobotContainer {
    private LoggedCommandController driverController;

    private static Elevator elevator;
    private static Arm arm;
    private static Intake intake;
    private static IntakeAngle intakeAngle;
    private static IntakeAligner intakeAligner;
    private static Outtake outtake;
    private static SwerveSubsystem swerveSubsystem;

    public RobotContainer() {
        switch (Constants.General.kRobotMode) {
            case REAL, SIM:
                arm = new Arm(true, new ArmIOController());
                elevator = new Elevator(true, new ElevatorIOController());
                intakeAngle = new IntakeAngle(true, new IntakeAngleIOController());
                intakeAligner = new IntakeAligner(true, new IntakeAlignerIOController());
                outtake = new Outtake(true, new OuttakeIOController());

                if(Constants.General.kRobotMode == Constants.RobotMode.REAL)
                    intake = new Intake(true, new IntakeIOController(), new LoggedDigitalInputIOReal(), 4);
                else
                    intake = new Intake(true, new IntakeIOController(), new LoggedDigitalInputIOSim(() -> driverController.options().getAsBoolean()), 4);

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(Constants.General.kDriverControllerPort));
                break;

            case REPLAY:
                arm = new Arm(true, new ArmIO() {});
                elevator = new Elevator(true, new ElevatorIO() {});
                intake = new Intake(true, new IntakeIO() {}, new LoggedDigitalInputIO() {}, 4);
                intakeAngle = new IntakeAngle(true, new IntakeAngleIO() {});
                intakeAligner = new IntakeAligner(true, new IntakeAlignerIO() {});
                outtake = new Outtake(true, new OuttakeIO() {});

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIO() {});
                break;
        }

        swerveSubsystem = new SwerveSubsystem(true);
        RobotStateBase.setInstance(new RobotState(Constants.Swerve.kSwerveConstants.chassis.kinematics));
        StateMachineBase.setInstance(new StateMachine());

        configureBindings();
    }

    public static Elevator getElevator() {
        return elevator;
    }

    public static Arm getArm() {
        return arm;
    }

    public static Intake getIntake() {
        return intake;
    }

    public static IntakeAngle getIntakeAngle() {
        return intakeAngle;
    }

    public static IntakeAligner getIntakeAligner() {
        return intakeAligner;
    }

    public static Outtake getOuttake() {
        return outtake;
    }

    public static SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    private void configureBindings() {
        driverController.cross().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.INTAKE_CORAL)));

        driverController.triangle().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.L1);
        }));

        driverController.povDown().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.RESET);
        }));
    }

    public void periodic() {
        driverController.periodic();

        if(Constants.General.kRobotMode == Constants.RobotMode.SIM)
            SimulatedArena.getInstance().simulationPeriodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
