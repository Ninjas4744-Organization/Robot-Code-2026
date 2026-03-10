package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.subsystem.IO;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class Box extends SubsystemBase implements
    ISubsystem.Resettable,
    ISubsystem.PositionControlled,
    ISubsystem.PercentControlled,
    ISubsystem.GoalOriented<Double>
{
    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;
    private BackgroundCommand backgroundCommand = new BackgroundCommand();

    public Box(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kBox);
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
        io.updateInputs(inputs);
        Logger.processInputs("Box", inputs);
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return inputs.LimitSwitch;
    }

    @Override
    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
            Commands.runOnce(() -> {
                backgroundCommand.stop();
                io.setPercent(-0.3);
            }),
            Commands.waitUntil(this::isReset),
            Commands.runOnce(io::stopMotor)
        );
    }

    @Override
    public boolean atGoal() {
        if (!enabled)
            return true;

        return inputs.AtGoal;
    }

    @Override
    public Double getGoal() {
        if (!enabled)
            return 0.0;

        return inputs.Goal;
    }

    public double getPosition() {
        if (!enabled)
            return 0.0;

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

    public void slowClose() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> io.setPercent(-0.1)),
            Commands.waitUntil(this::atGoal),
            Commands.runOnce(io::stopMotor)
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
}