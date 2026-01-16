package frc.robot.subsystems.intakeangle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAngleIO extends
    IO.BaseIO<IntakeAngleIOInputsAutoLogged>,
    IO.PositionControlled,
    IO.Encoder
{
    @AutoLog
    class IntakeAngleIOInputs extends Controller.ControllerIOInputs {
    }
}
