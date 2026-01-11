package frc.robot.subsystems.intakeangle;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.constants.SubsystemConstants;

public class IntakeAngleIOController implements IntakeAngleIO {
    private Controller controller;
    private CANcoder canCoder;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntakeAngle);

//        canCoder = new CANcoder(SubsystemConstants.kIntakeAngle.kCanCoderID);
        CANcoderConfiguration config = new CANcoderConfiguration();
//        config.MagnetSensor.MagnetOffset = SubsystemConstants.kIntakeAngle.kCanCoderOffset;
//        config.MagnetSensor.SensorDirection = SubsystemConstants.IntakeAngle.kCanCoderReversed;
        canCoder.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakeAngleIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);
        inputs.AbsoluteAngle = Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble() / 2);
        inputs.AtGoal = controller.atGoal();
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
    public void setPosition(double position) {
        controller.setPosition(position);
    }

    @Override
    public void setEncoder(double position) {
        controller.setEncoder(position);
    }
}