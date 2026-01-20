package frc.robot.subsystems.shooterindexer2.shooterindexer;

import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;

public class ShooterIndexer2IOController implements ShooterIndexer2IO {
    private Controller controller;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kShooterIndexer2);
    }

    @Override
    public void updateInputs(ShooterIndexer2IOInputsAutoLogged inputs) {
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