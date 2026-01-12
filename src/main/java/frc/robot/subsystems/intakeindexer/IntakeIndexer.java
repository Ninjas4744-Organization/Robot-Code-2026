package frc.robot.subsystems.intakeindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.SubsystemTools;
import org.littletonrobotics.junction.Logger;

public class IntakeIndexer extends SubsystemBase implements
        SubsystemTools.Periodicable,
        SubsystemTools.Resettable,
        SubsystemTools.Stoppable,
        SubsystemTools.VelocityControlled
{
    private IntakeIndexerIO io;
    private final IntakeIndexerIOInputsAutoLogged inputs = new IntakeIndexerIOInputsAutoLogged();
    private boolean enabled;

    public IntakeIndexer(boolean enabled) {
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
        Logger.processInputs("IntakeIndexer", inputs);
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