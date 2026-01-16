package frc.robot.subsystems.intakeangle;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;

public class IntakeAngleIOController implements IntakeAngleIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntakeAngle);
    }

    @Override
    public void updateInputs(IntakeAngleIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
    }

    @Override
    public void periodic() {
        controller.periodic();
    }

    @Override
    public void setPosition(double position) {
        controller.setPosition(position);
    }

    @Override
    public void setEncoder(double position) {
        controller.setEncoder(position);
    }
}