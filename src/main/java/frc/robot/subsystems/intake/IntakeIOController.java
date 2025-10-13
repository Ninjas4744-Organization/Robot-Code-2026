package frc.robot.subsystems.intake;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class IntakeIOController implements IntakeIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.Intake.kControllerConstants);
    }

    @Override
    public void updateInputs(IntakeIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }

    @Override
    public void setPercent(double percent) {
        controller.setPercent(percent);
    }

    @Override
    public void setVelocity(double velocity) {
        controller.setVelocity(velocity);
    }
}