package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.controllers.Controller;
import frc.robot.Robot;
import frc.robot.constants.SubsystemConstants;

public class ArmIOController implements ArmIO {
    private Controller controller;
    private CANcoder canCoder;

    @Override
    public void setup() {
        controller = Controller.createController(Controller.ControllerType.TalonFX, SubsystemConstants.kArm);

        if (Robot.isReal()) {
            canCoder = new CANcoder(SubsystemConstants.Other.kArmCANCoder.canCoderID);
            CANcoderConfiguration config = new CANcoderConfiguration();
            config.MagnetSensor.MagnetOffset = SubsystemConstants.Other.kArmCANCoder.canCoderOffset;
            config.MagnetSensor.SensorDirection = SubsystemConstants.Other.kArmCANCoder.canCoderReversed;
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