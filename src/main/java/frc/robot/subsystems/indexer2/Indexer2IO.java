package frc.robot.subsystems.indexer2;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface Indexer2IO extends
    IO.All<Indexer2IOInputsAutoLogged>
{
    @AutoLog
    class Indexer2IOInputs extends Controller.ControllerIOInputs {
    }
}
