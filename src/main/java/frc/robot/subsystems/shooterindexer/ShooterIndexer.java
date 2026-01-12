package frc.robot.subsystems.shooterindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.SubsystemTools;
import org.littletonrobotics.junction.Logger;

public class ShooterIndexer extends SubsystemBase implements
        SubsystemTools.Periodicable,
        SubsystemTools.VelocityControlled,
        SubsystemTools.Stoppable,
        SubsystemTools.Resettable
{
    private ShooterIndexerIO io;
    private final ShooterIndexerIOInputsAutoLogged inputs = new ShooterIndexerIOInputsAutoLogged();
    private boolean enabled;

    public ShooterIndexer(boolean enabled) {
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
        Logger.processInputs("ShooterIndexer", inputs);
    }

    public Command setVelocity(double velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(
            () -> io.setVelocity(velocity)
        );
    }

    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(0));
    }

    @Override
    public boolean isReset() {
        return false;
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stop();
    }
}