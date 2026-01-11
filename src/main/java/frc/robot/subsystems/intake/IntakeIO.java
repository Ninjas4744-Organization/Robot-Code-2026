package frc.robot.subsystems.intake;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void setPercent(double percent) {
    }

    default void setVelocity(double velocity) {
    }

    default void updateInputs(IntakeIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
