package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.subsystem.IO;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.Intake.IntakeStates.*;

public class Intake extends StateMachineBase<Intake.IntakeStates> {
    public enum IntakeStates {
        RESET,
        IDLE,
        INTAKE,
    }

    private IntakeStates state = IntakeStates.IDLE;

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

//    private boolean saveSystemCommand = false;
//    private int highCurrentFrames = 0;
//    private static final int HIGH_CURRENT_THRESHOLD_FRAMES = 12;
//    private BackgroundCommand saveCommand = new BackgroundCommand();
//    private double velocityBeforeSave;

    public Intake(boolean enabled) {
        super(IntakeStates.class);

        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntake);
            else
                this.io = new IO.All<>(){};
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

//        if (Math.abs(inputs.StatorCurrent) >= 25 && !saveSystemCommand) {
//            highCurrentFrames++;
//            if (highCurrentFrames >= HIGH_CURRENT_THRESHOLD_FRAMES) {
//                highCurrentFrames = 0;
//                saveSystemCommand = true;
//                velocityBeforeSave = inputs.Goal;
//                saveCommand.setNewTask(Commands.sequence(
//                    Commands.runOnce(() -> io.setPercent(-0.5)),
//                    Commands.waitSeconds(0.5),
//                    Commands.runOnce(() -> {
//                        io.setVelocity(velocityBeforeSave);
//                        saveSystemCommand = false;
//                    })
//                ));
//            }
//        } else {
//            highCurrentFrames = 0;
//        }

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
//        Logger.recordOutput("Intake/Save System Command", saveSystemCommand);
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> setVelocityCmd(0));

        addEdge(RESET, IDLE);

        addEdge(IDLE, INTAKE, setVelocityCmd(PositionsConstants.Intake.kIntake.get()));

        addEdge(INTAKE, IDLE, setVelocityCmd(0));

        addStateEnd(RESET, () -> true, IDLE);
    }

    public void setVelocity(double velocity) {
        if (!enabled)
            return;

        io.setVelocity(velocity);
    }

    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
    }

    public void stop() {
        if (!enabled)
            return;

//        saveSystemCommand = false;
//        saveCommand.stop();
        io.stopMotor();
    }

    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(inputs.Velocity) < 5;
    }

    public Command reset() {
        return stopCmd();
    }
}