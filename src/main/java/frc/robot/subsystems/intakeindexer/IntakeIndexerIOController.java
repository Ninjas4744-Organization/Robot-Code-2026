package frc.robot.subsystems.intakeindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;

public class IntakeIndexerIOController implements IntakeIndexerIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntakeIndexer);
    }

    @Override
    public void updateInputs(IntakeIndexerIOInputsAutoLogged inputs) {
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