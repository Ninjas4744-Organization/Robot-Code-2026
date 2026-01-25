package frc.robot.subsystems.climber;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO extends
    IO.All<ClimberIOInputsAutoLogged>
{
    @AutoLog
    class ClimberIOInputs extends Controller.ControllerIOInputs {
    }
}
