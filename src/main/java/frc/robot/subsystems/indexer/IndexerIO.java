package frc.robot.subsystems.indexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO extends IO.Controllable<IndexerIOInputsAutoLogged> {
    @AutoLog
    class IndexerIOInputs extends Controller.ControllerIOInputs {}
}
