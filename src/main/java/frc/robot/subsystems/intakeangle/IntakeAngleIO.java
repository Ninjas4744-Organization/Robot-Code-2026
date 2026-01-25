package frc.robot.subsystems.intakeangle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAngleIO extends
    IO.All<IntakeAngleIOInputsAutoLogged>
{
    @AutoLog
    class IntakeAngleIOInputs extends Controller.ControllerIOInputs {
    }
}
