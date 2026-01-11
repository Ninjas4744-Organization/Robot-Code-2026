package frc.robot.subsystems.intakeindexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class IntakeIndexer extends SubsystemBase {
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