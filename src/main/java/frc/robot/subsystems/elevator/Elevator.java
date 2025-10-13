package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

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

    public Command setHeight(DoubleSupplier wantedHeight) {
        if (!enabled) {
            return Commands.none();
        }
        return Commands.runOnce(() -> {
            io.setPosition(wantedHeight.getAsDouble());
        });
    }

    public Command close() {
        return setHeight(Constants.Elevator.Positions.Close::get);
    }

//    public Command goToLHeight(IntSupplier L) {
//        return switch (L.getAsInt()) {
//            case 1 -> setHeight(Constants.Elevator.Positions.Close::get);
//            case 2 -> setHeight(Constants.Elevator.Positions.L2::get);
//            case 3 -> setHeight(Constants.Elevator.Positions.L3::get);
//            case 4-> setHeight(Constants.Elevator.Positions.L4::get);
//            default -> Commands.none();
//        };
//    }

//    public Command goToAlgaeReefHeight() {
//        return setHeight(Constants.Elevator.Positions.AlgaeReef::get);
//    }
//
//    public Command goToNetHeight() {
//        return setHeight(Constants.Elevator.Positions.Net::get);
//    }
//
//    public Command goToSafeHeight() {
//        return setHeight(Constants.Elevator.Positions.Safe::get);
//    }
//
//    public Command goToIntakeHeight() {
//        return setHeight(Constants.Elevator.Positions.Intake::get);
//    }

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
//        return atGoal();
    }
}