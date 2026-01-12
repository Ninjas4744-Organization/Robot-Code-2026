package frc.robot.subsystems.intake;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO extends
        IO.Controllable<IntakeIOInputsAutoLogged>
{
    @AutoLog
    class IntakeIOInputs extends Controller.ControllerIOInputs {
    }
}
