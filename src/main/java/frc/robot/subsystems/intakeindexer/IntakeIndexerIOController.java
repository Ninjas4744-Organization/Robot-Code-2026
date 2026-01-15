package frc.robot.subsystems.intakeindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;

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
    public void setVelocity(double velocity) {
        controller.setVelocity(velocity);
    }

    @Override
    public void stopMotor() {
        controller.stop();
    }
}