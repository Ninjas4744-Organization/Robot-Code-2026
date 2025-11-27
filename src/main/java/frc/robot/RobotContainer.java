package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
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
        new VisionSubsystem();

//        RobotState.getInstance().setRobotPose(start);

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
        driverController.L1().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.IDLE)));

        driverController.R1().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.INTAKE_CORAL)));

        driverController.triangle().onTrue(Commands.runOnce(() -> {
            StateMachine.getInstance().changeRobotState(States.L1);
            StateMachine.getInstance().changeRobotState(States.L1_READY);
        }));

        driverController.povDown().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.RESET, true)));

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

        driverController.povUp().whileTrue(Commands.startEnd(() -> {
            outtake.forceKnowAlgaeInside(true);
        }, () -> {
            outtake.forceKnowAlgaeInside(false);
        }));

        driverController.circle().onTrue(Commands.runOnce(() -> StateMachine.getInstance().changeRobotState(States.INTAKE_ALGAE_REEF)));

//        driverController.R2().onTrue(Commands.either(
//                Commands.runOnce(() -> {
//                    double robotAngle = RobotState.getInstance().getRobotPose().getRotation().getDegrees();
//                    if (Math.abs(0 - robotAngle) < 90) {
//                        StateMachine.getInstance().changeRobotState(States.NET_INVERSE);
//                    } else {
//                        StateMachine.getInstance().changeRobotState(States.NET);
//                    }
//                }),
//                Commands.runOnce(() -> {
//                    double robotAngle = RobotState.getInstance().getRobotPose().getRotation().getDegrees();
//                    if (Math.abs(0 - robotAngle) < 90) {
//                        StateMachine.getInstance().changeRobotState(States.NET_INVERSE_READY);
//                    } else {
//                        StateMachine.getInstance().changeRobotState(States.NET_READY);
//                    }
//                }),
//                () -> RobotState.getInstance().getRobotState() == States.NET_READY ||
//                        RobotState.getInstance().getRobotState() == States.NET_INVERSE_READY
//        ));
    }

//    Pose2d start = new Pose2d(4, 4, Rotation2d.kZero);
//    Pose2d vision = new Pose2d(2, 2, Rotation2d.kZero);
//    double distSinceLastReset = 3;
//    double distFromTag = 3;
//    Random rand = new Random();
    public void periodic() {
        driverController.periodic();

        swerveSubsystem.swerveDrive(driverController::getLeftX, driverController::getLeftY, driverController::getRightX);

//        double a = 4;
//        Pose2d vision_rand = new Pose2d(vision.getX() + (rand.nextDouble() - 0.5) * (distFromTag * 0.0166 * 2),
//            vision.getY() + (rand.nextDouble() - 0.5) * (distFromTag * 0.0166 * 2),
//            vision.getRotation());
//        Logger.recordOutput("Shytt", vision_rand);
//        double visionStrength = 1 / (1 + a * Math.pow(distSinceLastReset, -0.5) * distFromTag * distFromTag);
//        Logger.recordOutput("Stren", visionStrength);
//        RobotState.getInstance().updateRobotPose(Swerve.getInstance().getModulePositions(), Swerve.getInstance().getGyro().getYaw());
//        if (DriverStation.isEnabled())
//            RobotState.getInstance().updateRobotPose(vision_rand, MathSharedStore.getTimestamp(), VecBuilder.fill(visionStrength, visionStrength, visionStrength / 2));

        if(Constants.General.kRobotMode == Constants.RobotMode.SIM)
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
}
