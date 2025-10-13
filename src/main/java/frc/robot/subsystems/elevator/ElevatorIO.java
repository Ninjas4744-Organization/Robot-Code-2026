package frc.robot.subsystems.elevator;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }

    default void setPosition(double position) {
    }

    default void setPercent(double position) {
    }

    default void setEncoder(double position) {
    }
}
