package frc.robot.subsystems.shooterindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIndexerIO extends IO.Controllable<ShooterIndexerIOInputsAutoLogged>{
    @AutoLog
    class ShooterIndexerIOInputs extends Controller.ControllerIOInputs {
    }
}
