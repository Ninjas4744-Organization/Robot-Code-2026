package frc.robot.subsystems.intakeindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIndexerIO {
    @AutoLog
    class IntakeIndexerIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void setPercent(double percent) {
    }

    default void setVelocity(double velocity) {
    }

    default void updateInputs(IntakeIndexerIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
