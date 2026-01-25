package frc.robot.subsystems.indexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;

public class IndexerIOController implements IndexerIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kIndexer);
    }

    @Override
    public void updateInputs(IndexerIOInputsAutoLogged inputs) {
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
    public void setVelocity(double velocity) {
        controller.setVelocity(velocity);
    }

    @Override
    public void setPercent(double percent) {
        controller.setPercent(percent);
    }

    @Override
    public void stopMotor() {
        controller.stop();
    }

    @Override
    public void setEncoder(double position) {
        controller.setEncoder(position);
    }
}