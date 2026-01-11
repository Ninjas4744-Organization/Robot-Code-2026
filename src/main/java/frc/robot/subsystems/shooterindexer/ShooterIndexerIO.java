package frc.robot.subsystems.shooterindexer;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIndexerIO {
    @AutoLog
    class ShooterIndexerIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void setPercent(double percent) {
    }

    default void setVelocity(double velocity) {
    }

    default void updateInputs(ShooterIndexerIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
