package frc.robot.subsystems.shooterindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;

public class ShooterIndexerIOController implements ShooterIndexerIO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kShooterIndexer);
    }

    @Override
    public void updateInputs(ShooterIndexerIOInputsAutoLogged inputs) {
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