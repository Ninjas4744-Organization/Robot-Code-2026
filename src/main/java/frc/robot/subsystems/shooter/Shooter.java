package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.GoalOriented<Double>,
        ISubsystem.Stoppable
{
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean enabled;
    private Command shooterCommand;

    public Shooter(boolean enabled, ShooterIO io) {
        this.enabled = enabled;

        if (enabled) {
            this.io = io;
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter Command", shooterCommand.isScheduled() && !shooterCommand.isFinished());
    }

    @Override
    public Command setVelocity(double velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setVelocity(velocity));
    }

    public Command createShooterCommand(DoubleSupplier velocity) {
        if (!enabled)
            return Commands.none();

        if (shooterCommand != null && shooterCommand.isScheduled() && !shooterCommand.isFinished())
            shooterCommand.cancel();

        shooterCommand = Commands.run(() -> io.setVelocity(velocity.getAsDouble()));

        return new DetachedCommand(shooterCommand);
    }

    @Override
    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
    }

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

    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            shooterCommand.cancel();
            io.stopMotor();
        });
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(inputs.Velocity) < 5 && (!shooterCommand.isScheduled() || shooterCommand.isFinished());
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stop();
    }
}