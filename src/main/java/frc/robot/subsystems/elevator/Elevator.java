package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private boolean enabled;

    public Elevator(boolean enabled, ElevatorIO io) {
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
        Logger.processInputs("Elevator", inputs);
    }

    public Command setHeight(double wantedHeight) {
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(() -> {
            io.setPosition(wantedHeight);
        });
    }

    public double getHeight() {
        return inputs.Position;
    }

    public boolean atGoal() {
        if (!enabled) {
            return true;
        }
        return inputs.AtGoal;
    }

    public Command reset() {
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(() -> io.setPercent(-0.4)).andThen(Commands.waitUntil(() -> inputs.LimitSwitch)).finallyDo(() -> io.setPercent(0));
    }

    public Command specialReset() {
        if (!enabled) {
            return Commands.none();
        }

        return setHeight(() -> getHeight() + 1.5).until(() -> atGoal() || inputs.Current > 55).andThen(() -> Commands.runOnce(() -> io.setPercent(0)));
    }

    public boolean isReset() {
        if (!enabled) {
            return true;
        }

        return inputs.LimitSwitch;
    }
}