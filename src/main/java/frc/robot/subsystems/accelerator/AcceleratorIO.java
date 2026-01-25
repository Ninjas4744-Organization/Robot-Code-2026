package frc.robot.subsystems.accelerator;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface AcceleratorIO extends
    IO.All<AcceleratorIOInputsAutoLogged>
{
    @AutoLog
    class AcceleratorIOInputs extends Controller.ControllerIOInputs {
    }
}
