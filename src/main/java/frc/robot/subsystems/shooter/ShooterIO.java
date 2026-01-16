package frc.robot.subsystems.shooter;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO extends
    IO.BaseIO<ShooterIOInputsAutoLogged>,
    IO.VelocityControlled,
    IO.Stoppable
{
    @AutoLog
    class ShooterIOInputs extends Controller.ControllerIOInputs {
    }
}
