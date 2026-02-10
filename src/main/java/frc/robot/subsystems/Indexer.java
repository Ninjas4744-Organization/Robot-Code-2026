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

public class Indexer extends SubsystemBase implements
    ISubsystem.Resettable,
    ISubsystem.VelocityControlled,
    ISubsystem.PercentControlled,
    ISubsystem.Stoppable
{
    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    public Indexer(boolean enabled) {
        this.enabled = enabled;
        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kIndexer);
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
        Logger.processInputs("Indexer", inputs);
    }

    @Override
    public void setVelocity(double velocity) {
        if (!enabled) return;
        io.setVelocity(velocity);
    }

    @Override
    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    @Override
    public double getVelocity() {
        return enabled ? inputs.Velocity : 0;
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
        return !enabled || Math.abs(inputs.Velocity) < 5;
    }

    @Override
    public Command reset() {
        return stopCmd();
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
}