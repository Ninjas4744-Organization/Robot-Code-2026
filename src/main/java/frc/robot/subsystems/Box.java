package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.subsystem.IO;
import frc.robot.RobotState;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.List;

import static frc.robot.subsystems.Box.BoxState.*;

public class Box extends StateMachineBase<Box.BoxState> {
    public enum BoxState {
        UNKNOWN,
        RESET,
        CLOSED,
        OPENED,
        SLOW_CLOSE,
        FORCE_CLOSE
    }

    private IO.All<ControllerIOInputsAutoLogged> ioLeft;
    private IO.All<ControllerIOInputsAutoLogged> ioRight;
    private final ControllerIOInputsAutoLogged inputsLeft = new ControllerIOInputsAutoLogged();
    private final ControllerIOInputsAutoLogged inputsRight = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    public Box(boolean enabled) {
        super(Box.BoxState.class);
        currentState = UNKNOWN;

        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay()) {
                this.ioLeft = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kBox);
                this.ioRight = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kRightBox);
            }
            else {
                this.ioLeft = new IO.All<>(){};
                this.ioRight = new IO.All<>(){};
            }

            ioLeft.setup();
            ioRight.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        ioLeft.periodic();
        ioRight.periodic();
        ioLeft.updateInputs(inputsLeft);
        ioRight.updateInputs(inputsRight);
        Logger.processInputs("Box Left", inputsLeft);
        Logger.processInputs("Box Right", inputsRight);

        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.sequence(
//            setPercentCmd(-0.3),
//            Commands.waitUntil(this::isReset),
//            setPositionCmd(PositionsConstants.Box.kClose.get())
//            resetEncoderCmd()
        ));

        addEdge(RESET, CLOSED);

        addEdge(List.of(CLOSED, SLOW_CLOSE), OPENED, () -> Commands.sequence(
            setPositionCmd(PositionsConstants.Box.kOpen.get()),
            Commands.waitUntil(this::atGoal)
        ));

//        addStateCommand(OPENED, Commands.run(() -> {
//            setPosition(PositionsConstants.Box.kOpen.get());
//        }));

        addEdge(OPENED, SLOW_CLOSE, Commands.sequence(
            setPercentCmd(-0.1),
            Commands.waitUntil(() -> inputsLeft.Position < 2),
            setPositionCmd(PositionsConstants.Box.kClose.get()),
            Commands.waitUntil(this::atGoal)
        ));

        addEdge(SLOW_CLOSE, CLOSED, Commands.sequence(
            setPositionCmd(PositionsConstants.Box.kClose.get()),
            Commands.waitUntil(this::atGoal)
        ));

        addEdge(List.of(OPENED, SLOW_CLOSE), FORCE_CLOSE, () -> Commands.sequence(
            setPositionCmd(PositionsConstants.Box.kClose.get()),
            Commands.waitUntil(this::atGoal)
        ));

        addEdge(FORCE_CLOSE, CLOSED);


        addStateEnd(RESET, () -> !DriverStation.isTest(), CLOSED);

        addStateEnd(SLOW_CLOSE, () -> !DriverStation.isTest(), CLOSED);

        addStateEnd(CLOSED,
            () -> !DriverStation.isTest() && RobotState.get().getRobotPose().getX() > PositionsConstants.Swerve.kNeutralXThreshold.get(),
            OPENED
        );

        addStateEnd(FORCE_CLOSE, () -> true, CLOSED);
    }

    public boolean atGoal() {
        if (!enabled)
            return true;

        return inputsLeft.AtGoal && inputsRight.AtGoal;
    }

    private void setPosition(double position) {
        if (!enabled)
            return;

        ioLeft.setPosition(position);
        ioRight.setPosition(position);
    }

    private Command setPositionCmd(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public void setPercent(double percent) {
        if (!enabled)
            return;

        ioLeft.setPercent(percent);
        ioRight.setPercent(percent * 0.8);
    }

    public Command setPercentCmd(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    public void stop() {
        if (!enabled)
            return;

        ioLeft.stopMotor();
        ioRight.stopMotor();
    }

    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    public void resetEncoder() {
        if (!enabled)
            return;

        ioLeft.setEncoder(0);
        ioRight.setEncoder(0);
    }

    public Command resetEncoderCmd() {
        return Commands.runOnce(this::resetEncoder);
    }
}