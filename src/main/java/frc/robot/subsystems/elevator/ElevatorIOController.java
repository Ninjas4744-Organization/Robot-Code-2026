package frc.robot.subsystems.elevator;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;

public class ElevatorIOController implements ElevatorIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.Elevator.kControllerConstants);
    }

    @Override
    public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
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
    public void setPercent(double position) {
        controller.setPercent(position);
    }

    @Override
    public void setEncoder(double position) {
        controller.setEncoder(position);
    }
}