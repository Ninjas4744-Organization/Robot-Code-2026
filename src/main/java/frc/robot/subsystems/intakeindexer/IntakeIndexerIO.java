package frc.robot.subsystems.intakeindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIndexerIO extends
    IO.BaseIO<IntakeIndexerIOInputsAutoLogged>,
    IO.VelocityControlled,
    IO.Stoppable
{
    @AutoLog
    class IntakeIndexerIOInputs extends Controller.ControllerIOInputs {
    }
}
