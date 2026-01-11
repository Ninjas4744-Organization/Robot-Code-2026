package frc.robot.subsystems.shooterindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class ShooterIndexer extends SubsystemBase {
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

    public Command setVelocity(DoubleSupplier velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(
            () -> io.setVelocity(velocity.getAsDouble())
        );
    }

    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(0));
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stop();
    }
}