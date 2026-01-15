package frc.robot.subsystems.climberangle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberAngleIO extends
        IO.BaseIO<ClimberAngleIOInputsAutoLogged>,
        IO.PositionControlled,
        IO.VelocityControlled,
        IO.Stoppable
{
    @AutoLog
    class ClimberAngleIOInputs extends Controller.ControllerIOInputs {

    }
}