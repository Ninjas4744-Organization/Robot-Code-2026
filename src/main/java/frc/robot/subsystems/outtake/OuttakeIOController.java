package frc.robot.subsystems.outtake;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class OuttakeIOController implements OuttakeIO{
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.Outtake.kControllerConstants);
    }

    @Override
    public void updateInputs(OuttakeIOInputsAutoLogged inputs) {
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
}
