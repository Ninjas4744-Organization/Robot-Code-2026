package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs extends Controller.ControllerIOInputs {
        Rotation2d AbsoluteAngle = Rotation2d.fromDegrees(90);
    }

    default void setup() {
    }

    default void updateInputs(ArmIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }

    default void setPosition(Rotation2d position){
    }

    default void stop() {
    }

    default void setEncoder(double position){
    }
}
