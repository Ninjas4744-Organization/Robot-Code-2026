package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.subsystem.IO;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase implements
    ISubsystem.Resettable,
    ISubsystem.PositionControlled,
    ISubsystem.PercentControlled,
    ISubsystem.GoalOriented<Double>,
    ISubsystem.Stoppable
{
    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    public Climber(boolean enabled) {
        this.enabled = enabled;
        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kClimber);
            else
                this.io = new IO.All<>(){};
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    @Override
    public void stop() {
        if (!enabled) return;
        io.stopMotor();
    }

    @Override
    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    @Override
    public boolean isReset() {
        return !enabled || inputs.LimitSwitch || GeneralConstants.kRobotMode.isSim();
    }

    @Override
    public Command reset() {
        if (!enabled) return Commands.none();
        return Commands.runOnce(() -> io.setPercent(-0.4))
            .andThen(Commands.waitUntil(this::isReset))
            .finallyDo(this::stop);
    }

    @Override
    public void setPosition(double position) {
        if (!enabled) return;
        io.setPosition(position);
    }

    @Override
    public Command setPositionCmd(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    @Override
    public double getPosition() {
        return enabled ? inputs.Position : 0;
    }

    @Override
    public void setPercent(double percent) {
        if (!enabled) return;
        io.setPercent(percent);
    }

    @Override
    public Command setPercentCmd(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    @Override
    public double getOutput() {
        return enabled ? inputs.Output : 0;
    }

    @Override
    public boolean atGoal() {
        return !enabled || inputs.AtGoal;
    }

    @Override
    public Double getGoal() {
        return enabled ? inputs.Goal : 0.0;
    }
}