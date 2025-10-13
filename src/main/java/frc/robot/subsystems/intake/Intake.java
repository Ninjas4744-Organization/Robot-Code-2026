package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInput;
import frc.lib.NinjasLib.loggeddigitalinput.LoggedDigitalInputIO;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private boolean enabled;
    private LoggedDigitalInput beamBreaker;
//    private boolean isCoralInside = false;
//    private Timer currentTimer = new Timer();

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
//        return isCoralInside;
//        return false;
    }

    public Command intake() {
        return setVelocity(Constants.Intake.Speeds.Intake::get);
    }

    public Command outtake() {
        return Commands.sequence(
                setVelocity(Constants.Intake.Speeds.Outtake::get)
        );
    }

    public Command outtakeL1() {
        return setVelocity(Constants.Intake.Speeds.OuttakeL1::get);
    }

    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(0));
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
//                intake(),
//                Commands.race(
//                        Commands.waitUntil(() -> {
//                            if (Math.abs(inputs.Current) > 65) {
//                                if (!currentTimer.isRunning())
//                                    currentTimer.restart();
//                            } else {
//                                currentTimer.stop();
//                                currentTimer.reset();
//                            }
//
//                            if(currentTimer.get() > 0.25)
//                                isCoralInside = true;
//                            return isCoralInside;
//                        }),
//                        Commands.waitSeconds(0.5)
//                ),
                stop()
        );
    }
}