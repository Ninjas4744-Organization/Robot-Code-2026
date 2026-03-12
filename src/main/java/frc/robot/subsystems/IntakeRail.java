package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.subsystem.IO;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeRail extends StateMachineBase
{
    public enum IntakeRailStates {
        RESET,
        CLOSED,
        OPENED,
        SLOW_CLOSE,
        SAVE_OPEN
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;
    private BackgroundCommand backgroundCommand = new BackgroundCommand();

    private boolean saveSystemCommand = false;
    private int highCurrentFrames = 0;
    private static final int HIGH_CURRENT_THRESHOLD_FRAMES = 12;
    private boolean isResetting = false;

    public IntakeRail(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntakeOpen);
            else
                this.io = new IO.All<>() {};
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        if (Math.abs(inputs.StatorCurrent) >= 25 && !inputs.LimitSwitch && !saveSystemCommand && !isResetting) {
            highCurrentFrames++;
            if (highCurrentFrames >= HIGH_CURRENT_THRESHOLD_FRAMES) {
                highCurrentFrames = 0;
                saveSystemCommand = true;
                manualOpen();
            }
        } else {
            highCurrentFrames = 0;
        }

        io.updateInputs(inputs);
        Logger.processInputs("Intake Open", inputs);
        Logger.recordOutput("Intake Open/Save System Command", saveSystemCommand);
    }

    public boolean isReset() {
        return !enabled || inputs.LimitSwitches[0];
    }

    public Command reset() {
        if (!enabled) return Commands.none();
        return Commands.runOnce(() -> {
            isResetting = true;
            backgroundCommand.stop();
            saveSystemCommand = false;
            io.setPercent(-0.5);
        })
        .andThen(Commands.waitUntil(this::isReset))
        .finallyDo(() -> {
            io.stopMotor();
            isResetting = false;
        });
    }

    public boolean atGoal() {
//        return !enabled || inputs.LimitSwitches[goal];
        return !enabled || inputs.AtGoal;
    }

    public Double getGoal() {
        return enabled ? inputs.Goal : 0;
    }

    public void manualOpen() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> io.setPercent(0.5)),
            Commands.waitUntil(() -> inputs.LimitSwitches[1]),
            Commands.runOnce(() -> {
                io.stopMotor();
                saveSystemCommand = false;
            })
        ));
    }

    public void manualClose() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> io.setPercent(-0.5)),
            Commands.waitUntil(() -> inputs.LimitSwitches[0]),
            Commands.runOnce(() -> io.setPosition(PositionsConstants.IntakeOpen.kOpen.get()))
        ));
    }

    public double getOutput() {
        if (!enabled)
            return 0;

        return inputs.Output;
    }

    public void setPercent(double percent) {
        if (!enabled)
            return;

        io.setPercent(percent);
    }

    public Command setPercentCmd(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    public double getPosition() {
        if (!enabled)
            return 0;

        return inputs.Position;
    }

    public void setPosition(double position) {
        if (!enabled)
            return;

        backgroundCommand.stop();
        io.setPosition(position);
    }

    public Command setPositionCmd(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }
}