package frc.robot.subsystems;

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
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    public Box(boolean enabled) {
        super(Box.BoxState.class);
        currentState = UNKNOWN;

        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kBox);
            else
                this.io = new IO.All<>(){};

            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Box", inputs);

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

        addEdge(OPENED, SLOW_CLOSE, Commands.sequence(
            setPercentCmd(-0.2),
            Commands.waitUntil(() -> inputs.Position < 5),
            setPositionCmd(PositionsConstants.Box.kClose.get())
        ));

        addEdge(SLOW_CLOSE, CLOSED);


        addStateEnd(RESET, () -> true, CLOSED);

        addStateEnd(SLOW_CLOSE, () -> true, CLOSED);

        addStateEnd(CLOSED,
            () -> RobotState.get().getRobotPose().getX() > PositionsConstants.Swerve.kNeutralXThreshold.get(),
            OPENED
        );
    }

    public boolean atGoal() {
        if (!enabled)
            return true;

        return inputs.AtGoal;
    }

    private void setPosition(double position) {
        if (!enabled)
            return;

        io.setPosition(position);
    }

    private Command setPositionCmd(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public void setPercent(double percent) {
        if (!enabled)
            return;

        io.setPercent(percent);
    }

    public Command setPercentCmd(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    public void stop() {
        if (!enabled)
            return;

        io.stopMotor();
    }

    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    public void resetEncoder() {
        if (!enabled)
            return;

        io.setEncoder(0);
    }

    public Command resetEncoderCmd() {
        return Commands.runOnce(this::resetEncoder);
    }
}