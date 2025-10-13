package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Constants;
import frc.robot.Robot;

public class ArmIOController implements ArmIO {
    private Controller controller;
    private CANcoder canCoder;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, Constants.Arm.kControllerConstants);

        if (Robot.isReal()) {
            canCoder = new CANcoder(Constants.Arm.kCanCoderID);
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.MagnetOffset = Constants.Arm.kCanCoderOffset;
            config.MagnetSensor.SensorDirection = Constants.Arm.kCanCoderReversed;
            canCoder.getConfigurator().apply(config);
        }
    }

    @Override
    public void updateInputs(ArmIOInputsAutoLogged inputs) {
        controller.updateInputs(inputs);

        if (Robot.isReal())
            inputs.AbsoluteAngle = Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble() * 2);
        else
            inputs.AbsoluteAngle = Rotation2d.fromRotations(controller.getPosition());
    }

    @Override
    public void periodic() {
        controller.periodic();
    }

    @Override
    public void setPosition(Rotation2d position) {
        controller.setPosition(position.getRotations());
    }

    @Override
    public void stop() {
        controller.stop();
    }

    @Override
    public void setEncoder(double position) {
        controller.setEncoder(position);
    }
}