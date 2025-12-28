package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
import frc.robot.subsystems.VisionSubsystem;
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
import org.littletonrobotics.junction.Logger;

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
            case WORKSHOP, COMP, SIM, SIM_COMP:
                arm = new Arm(true, new ArmIOController());
                elevator = new Elevator(true, new ElevatorIOController());
                intakeAngle = new IntakeAngle(true, new IntakeAngleIOController());
                intakeAligner = new IntakeAligner(true, new IntakeAlignerIOController());
                outtake = new Outtake(true, new OuttakeIOController());

                if(Constants.General.kRobotMode.isReal())
                    intake = new Intake(true, new IntakeIOController(), new LoggedDigitalInputIOReal(), 4);
                else
                    intake = new Intake(true, new IntakeIOController(), new LoggedDigitalInputIOSim(() -> driverController.options().getAsBoolean()), 4);

                driverController = new LoggedCommandController("Driver", new LoggedCommandControllerIOPS5(Constants.General.kDriverControllerPort));
                break;

            case REPLAY, REPLAY_COMP:
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
        new VisionSubsystem();

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
        driverController.R1().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.INTAKE_CORAL)));
        driverController.L1().onTrue(Commands.runOnce(() -> {
            System.out.println("L1 Button");
            if(intake.isCoralInside()) {
                System.out.println("coral in intake inside");
                StateMachine.getInstance().changeRobotState(States.CORAL_IN_INTAKE);
            } else if(outtake.isCoralInside()) {
                System.out.println("coral in outtake inside");
                StateMachine.getInstance().changeRobotState(States.CORAL_IN_OUTTAKE);
            } else if(outtake.isAlgaeInside()) {
                System.out.println("algae inside");
                StateMachine.getInstance().changeRobotState(States.ALGAE_IN_OUTTAKE);
            } else
                StateMachine.getInstance().changeRobotState(States.IDLE);
        }));

        driverController.R2().onTrue(Commands.runOnce(() -> {
            RobotState.setReefSide(true);
            StateMachine.getInstance().changeRobotState(States.DRIVE_REEF);
        }));

        driverController.L2().onTrue(Commands.runOnce(() -> {
            RobotState.setReefSide(false);
            StateMachine.getInstance().changeRobotState(States.DRIVE_REEF);
        }));

        driverController.create().onTrue(Commands.runOnce(() -> {
            RobotState.setL(RobotState.getL() % 4 + 1);
        }));

        driverController.square().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.INTAKE_ALGAE_FLOOR);

            double robotAngle = RobotState.getInstance().getRobotPose().getRotation().getDegrees();
            if (Math.abs(0 - robotAngle) < 90) {
                StateMachine.getInstance().changeRobotState(States.NET_INVERSE_READY);
                StateMachine.getInstance().changeRobotState(States.NET_INVERSE);
            } else {
                StateMachine.getInstance().changeRobotState(States.NET_READY);
                StateMachine.getInstance().changeRobotState(States.NET);
            }
        }));
        driverController.circle().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.INTAKE_ALGAE_REEF)));

        driverController.povDown().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(Rotation2d.kZero)));
        driverController.povLeft().onTrue(Commands.runOnce(() -> RobotState.getInstance().resetGyro(RobotState.getInstance().getRobotPose().getRotation())));
        driverController.povRight().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.RESET, true)));
        driverController.povUp().toggleOnTrue(Commands.startEnd(() -> outtake.forceKnowAlgaeInside(true), () -> outtake.forceKnowAlgaeInside(false)));
    }

    public void periodic() {
        driverController.periodic();

        swerveSubsystem.swerveDrive(driverController::getLeftX, driverController::getLeftY, driverController::getRightX);

        if(Constants.General.kRobotMode.isSim())
            SimulatedArena.getInstance().simulationPeriodic();

        double elevatorStroke = 1.415;
        double elevator1 = elevator.getHeight() * (elevatorStroke / 10.8);
        double elevator0 = Math.max(elevator1 - 0.58, 0);
        Logger.recordOutput("Simulation Poses/Intake", new Pose3d(0, -0.312, 0.18, new Rotation3d(-intakeAngle.getAngle().getRadians() - Math.PI - Units.degreesToRadians(15), 0, 0)));
        Logger.recordOutput("Simulation Poses/Elevator 0", new Pose3d(0, 0, elevator0, new Rotation3d()));
        Logger.recordOutput("Simulation Poses/Elevator 1", new Pose3d(0, 0, elevator1, new Rotation3d()));
        Logger.recordOutput("Simulation Poses/Arm", new Pose3d(0, 0.12, 0.3835 + elevator1, new Rotation3d(Units.degreesToRadians(90), -arm.getAngle().getRadians() + Math.PI / 2, 0)));
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }

    public void reset() {
        if (Constants.General.kRobotMode.isComp()) {
            RobotState.getInstance().setRobotState(States.STARTING_POSE);
            StateMachine.getInstance().changeRobotState(States.IDLE, true);
        }
        else {
            RobotState.getInstance().setRobotState(States.UNKNOWN);
            StateMachine.getInstance().changeRobotState(States.RESET, true);
        }
    }
}
