package frc.robot.subsystems.indexer2;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import org.littletonrobotics.junction.Logger;

public class Indexer2 extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.PercentControlled,
        ISubsystem.Stoppable
{
    private Indexer2IO io;
    private final Indexer2IOInputsAutoLogged inputs = new Indexer2IOInputsAutoLogged();
    private boolean enabled;

    public Indexer2(boolean enabled, Indexer2IO io) {
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
        Logger.processInputs("Indexer2", inputs);
    }

    public Command setVelocity(double velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setVelocity(velocity));
    }

    @Override
    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
    }

    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(io::stopMotor);
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

    @Override
    public Command setPercent(double percent) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(percent));
    }

    @Override
    public double getOutput() {
        if (!enabled)
            return 0;

        return inputs.Output;
    }
}