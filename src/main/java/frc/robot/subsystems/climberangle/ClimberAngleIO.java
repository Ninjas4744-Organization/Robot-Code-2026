package frc.robot.subsystems.climberangle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberAngleIO extends
    IO.All<ClimberAngleIOInputsAutoLogged>
{
    @AutoLog
    class ClimberAngleIOInputs extends Controller.ControllerIOInputs {
    }
}