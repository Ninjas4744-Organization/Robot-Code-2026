package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandController;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIO;
import frc.lib.NinjasLib.loggedcontroller.LoggedCommandControllerIOPS5;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.example.ExampleSubsystemIO;
import frc.robot.subsystems.example.ExampleSubsystemIOController;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
    private LoggedCommandController driverController;
    private LoggedCommandController operatorController;

    private ExampleSubsystem exampleSubsystem;

    public RobotContainer() {
        RobotStateBase.setInstance(new RobotState());

        switch (Constants.General.kRobotMode) {
            case REAL, SIM:
                exampleSubsystem = new ExampleSubsystem(false, new ExampleSubsystemIOController());

                driverController = new LoggedCommandController(new LoggedCommandControllerIOPS5(Constants.General.kDriverControllerPort));
                operatorController = new LoggedCommandController(new LoggedCommandControllerIOPS5(Constants.General.kOperatorControllerPort));
                break;

            case REPLAY:
                exampleSubsystem = new ExampleSubsystem(false, new ExampleSubsystemIO() {});

                driverController = new LoggedCommandController(new LoggedCommandControllerIO() {});
                operatorController = new LoggedCommandController(new LoggedCommandControllerIO() {});
                break;
        }

        configureBindings();
    }

    public ExampleSubsystem getExampleSubsystem() {
        return exampleSubsystem;
    }

    private void configureBindings() {
        driverController.cross().onTrue(exampleSubsystem.setAngle(() -> Rotation2d.kCCW_90deg));
        driverController.circle().onTrue(exampleSubsystem.setAngle(() -> Rotation2d.kZero));
    }

    public void periodic() {
        Logger.recordOutput("Output1", 5);

        if(Constants.General.kRobotMode == Constants.RobotMode.SIM)
            SimulatedArena.getInstance().simulationPeriodic();
    }

    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
