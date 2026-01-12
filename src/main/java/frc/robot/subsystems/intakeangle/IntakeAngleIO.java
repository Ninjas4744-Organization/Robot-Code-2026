package frc.robot.subsystems.intakeangle;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.subsystem_interfaces.IO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAngleIO extends IO.Controllable<IntakeAngleIOInputsAutoLogged> {
    @AutoLog
    class IntakeAngleIOInputs extends Controller.ControllerIOInputs {
        Rotation2d AbsoluteAngle = Rotation2d.kZero;
    }
}
