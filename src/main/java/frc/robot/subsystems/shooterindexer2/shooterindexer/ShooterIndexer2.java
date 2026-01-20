package frc.robot.subsystems.shooterindexer2.shooterindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import org.littletonrobotics.junction.Logger;

public class ShooterIndexer2 extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.Stoppable
{
    private ShooterIndexer2IO io;
    private final ShooterIndexer2IOInputsAutoLogged inputs = new ShooterIndexer2IOInputsAutoLogged();
    private boolean enabled;

    public ShooterIndexer2(boolean enabled, ShooterIndexer2IO io) {
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
        Logger.processInputs("Shooter Indexer 2", inputs);
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
}