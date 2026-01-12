package frc.robot.subsystems.intakeindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIndexerIO extends IO.Controllable<IntakeIndexerIOInputsAutoLogged>{
    @AutoLog
    class IntakeIndexerIOInputs extends Controller.ControllerIOInputs {
    }
}
