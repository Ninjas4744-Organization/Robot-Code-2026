package frc.robot.subsystems.shooterindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIndexerIO extends
    IO.BaseIO<ShooterIndexerIOInputsAutoLogged>,
    IO.VelocityControlled,
    IO.Stoppable
{
    @AutoLog
    class ShooterIndexerIOInputs extends Controller.ControllerIOInputs {
    }
}
