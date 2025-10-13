package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInput;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIO;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean enabled;
    private LoggedDigitalInput beamBreaker;

    public Intake(boolean enabled, IntakeIO io, LoggedDigitalInputIO beamBreakerIO, int beamBreakerPort) {
        this.enabled = enabled;
        beamBreaker = new LoggedDigitalInput("Intake/Beam Breaker", beamBreakerPort, enabled, false, true, beamBreakerIO);

        if (enabled) {
            this.io = io;
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        beamBreaker.periodic();

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
    }

    public Command setVelocity(DoubleSupplier velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(
            () -> io.setVelocity(velocity.getAsDouble())
        );
    }

    public boolean isCoralInside() {
        if (!enabled)
            return true;

        return beamBreaker.get();
    }

    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(0));
    }

    public Command reset() {
        return stop();
    }
}