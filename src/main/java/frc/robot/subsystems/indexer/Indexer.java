package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.PercentControlled,
        ISubsystem.Stoppable
{
    private IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();
    private boolean enabled;

    public Indexer(boolean enabled, IndexerIO io) {
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
        Logger.processInputs("Indexer", inputs);
    }

    @Override
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

    @Override
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

    @Override
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