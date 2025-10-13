package frc.robot.subsystems.intakealigner;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class IntakeAligner extends SubsystemBase {
    private IntakeAlignerIO io;
    private final IntakeAlignerIOInputsAutoLogged inputs = new IntakeAlignerIOInputsAutoLogged();
    private boolean enabled;

    public IntakeAligner(boolean enabled, IntakeAlignerIO io) {
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
        Logger.processInputs("IntakeAligner", inputs);
    }

//    public Command setPercent(DoubleSupplier percent) {
//        if (!enabled) {
//            return Commands.none();
//        }
//
//        return Commands.runOnce(
//                () -> io.setPercent(percent.getAsDouble())
//        );
//    }

    public Command align() {
        if(!enabled)
            return Commands.none();
//        return setPercent(Constants.IntakeAligner.Speeds.Align::get);
        return Commands.runOnce(() -> io.setVelocity(Constants.IntakeAligner.Speeds.Align.get()));
    }

    public Command stop() {
        if(!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(0));
    }
}