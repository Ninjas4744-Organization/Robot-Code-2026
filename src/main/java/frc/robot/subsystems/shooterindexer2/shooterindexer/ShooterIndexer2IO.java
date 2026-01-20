package frc.robot.subsystems.shooterindexer2.shooterindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIndexer2IO extends
    IO.BaseIO<ShooterIndexer2IOInputsAutoLogged>,
    IO.VelocityControlled,
    IO.Stoppable
{
    @AutoLog
    class ShooterIndexer2IOInputs extends Controller.ControllerIOInputs {
    }
}
