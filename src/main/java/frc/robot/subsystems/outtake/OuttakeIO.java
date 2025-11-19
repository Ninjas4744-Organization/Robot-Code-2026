package frc.robot.subsystems.outtake;

import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface OuttakeIO {
    @AutoLog
    class OuttakeIOInputs extends Controller.ControllerIOInputs {
        boolean isCoralInside = false;
        boolean isAlgaeInside = false;
    }

    default void setup() {
    }

    default void updateInputs(OuttakeIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }

    default void setPercent(double percent){
    }
}
