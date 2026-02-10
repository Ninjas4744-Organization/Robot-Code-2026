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
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.GoalOriented<Double>,
        ISubsystem.Stoppable
{
    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;
    private BackgroundCommand backgroundCommand;

    public Shooter(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kShooter);
            else
                this.io = new IO.All<>(){};
            io.setup();

            backgroundCommand = new BackgroundCommand();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/Shooter Command", backgroundCommand.isRunning());
    }

    @Override
    public void setVelocity(double velocity) {
        if (!enabled)
            return;

        io.setVelocity(velocity);
    }

    @Override
    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    @Override
    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
    }

    @Override
    public boolean atGoal(){
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

    @Override
    public void stop() {
        if (!enabled)
            return;

        backgroundCommand.stop();
        io.stopMotor();
    }

    @Override
    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(inputs.Velocity) < 5 && !backgroundCommand.isRunning();
    }

    @Override
    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stopCmd();
    }

    public void autoHubVelocity() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.run(() -> {
            io.setVelocity(PositionsConstants.Shooter.getShootSpeed(RobotState.getInstance().getDistToHub()));
        }));
    }

    public void autoDeliveryVelocity() {
        if (!enabled)
            return;

        backgroundCommand.setNewTask(Commands.run(() -> {
            double dist = RobotState.getInstance().getDistance(PositionsConstants.Swerve.getDeliveryTarget());
            io.setVelocity(PositionsConstants.Shooter.getDeliverySpeed(dist));
        }));
    }
}