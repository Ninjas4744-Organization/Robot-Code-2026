package frc.robot.subsystems.shooter;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOInputs extends Controller.ControllerIOInputs {
    }

    default void setup() {
    }

    default void setPercent(double percent) {
    }

    default void setVelocity(double velocity) {
    }

    default void atGoal(){
    }

    default void updateInputs(ShooterIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }
}
