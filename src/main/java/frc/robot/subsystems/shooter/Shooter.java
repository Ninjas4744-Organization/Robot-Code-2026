package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Shooter extends SubsystemBase {
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean enabled;

    public Shooter(boolean enabled) {
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
        Logger.processInputs("Shooter", inputs);
    }

    public Command setVelocity(DoubleSupplier velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(
            () -> io.setVelocity(velocity.getAsDouble())
        );
    }

    public boolean atGoal(){
        if (!enabled){
            return true;
        }
        return inputs.AtGoal;
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