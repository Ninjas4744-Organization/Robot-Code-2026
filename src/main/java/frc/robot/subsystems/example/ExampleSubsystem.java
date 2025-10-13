package frc.robot.subsystems.example;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ExampleSubsystem extends SubsystemBase {
    private ExampleSubsystemIO io;
    private final ExampleSubsystemIOInputsAutoLogged inputs = new ExampleSubsystemIOInputsAutoLogged();
    private boolean enabled;

    public ExampleSubsystem(boolean enabled, ExampleSubsystemIO io) {
        if (enabled) {
            this.io = io;
            io.setup();
        }
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        io.updateInputs(inputs);
        Logger.processInputs("ExampleSubsystem", inputs);
    }

    public Command setAngle(Supplier<Rotation2d> angle) {
        return Commands.runOnce(() -> io.setAngle(angle.get()));
    }

    public Command setPercent(DoubleSupplier percent) {
        return Commands.runOnce(() -> io.setPercent(percent.getAsDouble()));
    }
}
