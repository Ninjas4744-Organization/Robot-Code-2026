package frc.robot.subsystems.intakeangle;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeAngleIO {
    @AutoLog
    class IntakeAngleIOInputs extends Controller.ControllerIOInputs {
        Rotation2d AbsoluteAngle = Rotation2d.kZero;
    }

    default void setup() {
    }

    default void updateInputs(IntakeAngleIOInputsAutoLogged inputs) {
    }

    default void periodic() {
    }

    default void setPercent(double percent){
    }

    default void setPosition(double position){
    }

    default void setEncoder(double position){
    }
}
