package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.GoalOriented<Double>,
        ISubsystem.Stoppable
{
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean enabled;
    private Command updatingVelocityCommand;

    public Shooter(boolean enabled, ShooterIO io) {
        this.enabled = enabled;

        if (enabled) {
            this.io = io;
            io.setup();
        }

        updatingVelocityCommand = Commands.run(() -> io.setVelocity(PositionsConstants.Shooter.getShootSpeed(FieldConstants.getDistToHub())));
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Updating Velocity Command", updatingVelocityCommand.isScheduled() && !updatingVelocityCommand.isFinished());
    }

    @Override
    public Command setVelocity(double velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setVelocity(velocity));
    }

    public Command startUpdatingVelocity() {
        if (!enabled)
            return Commands.none();

        return new DetachedCommand(updatingVelocityCommand);
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
            updatingVelocityCommand.cancel();
            io.stopMotor();
        });
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(inputs.Velocity) < 5;
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stop();
    }
}