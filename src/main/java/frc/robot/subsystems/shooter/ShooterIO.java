package frc.robot.subsystems.shooter;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends IO.Controllable<ShooterIOInputsAutoLogged> {
    @AutoLog
    class ShooterIOInputs extends Controller.ControllerIOInputs {
    }
}
