package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.subsystem.IO;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.robot.RobotState;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeOpen extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.GoalOriented<Integer>
{
    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;
    private int goal = 0;

    private BackgroundCommand backgroundCommand = new BackgroundCommand();

    public IntakeOpen(boolean enabled) {
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
        if (!enabled) return;
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake Open", inputs);
    }

    @Override
    public boolean isReset() {
        return !enabled || inputs.LimitSwitches[0] || GeneralConstants.kRobotMode.isSim();
    }

    @Override
    public Command reset() {
        if (!enabled) return Commands.none();
        return Commands.runOnce(() -> {
            goal = 0;
            backgroundCommand.stop();
            io.setPercent(-0.3);
        })
        .andThen(Commands.waitUntil(this::isReset))
        .finallyDo(io::stopMotor);
    }

    @Override
    public boolean atGoal() {
        return !enabled || inputs.LimitSwitches[goal];
    }

    @Override
    public Integer getGoal() {
        return enabled ? goal : 0;
    }

    public void open() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> goal = 1),
            Commands.runOnce(() -> io.setPercent(0.75)),
            Commands.waitUntil(() -> inputs.LimitSwitches[1]),
            Commands.runOnce(io::stopMotor)
        ));
    }

    public void close() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> goal = 0),
            Commands.runOnce(() -> io.setPercent(-0.75)),
            Commands.waitUntil(() -> inputs.LimitSwitches[0]),
            Commands.runOnce(io::stopMotor)
        ));
    }

    public void shootClose() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.sequence(
            Commands.runOnce(() -> goal = 0),
            Commands.runOnce(() -> io.setPercent(-0.75)),
            Commands.waitUntil(() -> inputs.LimitSwitches[0]),
            Commands.runOnce(io::stopMotor),
            Commands.runOnce(() -> RobotState.setIntake(true))
        ));
    }
}