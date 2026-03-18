package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.subsystem.IO;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;

import static frc.robot.subsystems.IntakeRail.IntakeRailState.*;

public class IntakeRail extends StateMachineBase<IntakeRail.IntakeRailState> {
    public enum IntakeRailState {
        UNKNOWN,
        RESET,
        CLOSED,
        OPENED,
        SLOW_CLOSE,
        SAVE_OPEN,
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    private int highCurrentFrames = 0;
    private static final int kHighCurrentThreshold = 12;
    private static final int kHighCurrentThresholdFrames = 12;

    public IntakeRail(boolean enabled) {
        super(IntakeRailState.class);
        currentState = UNKNOWN;

        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntakeRail);
            else
                this.io = new IO.All<>() {};

            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();

        if (Math.abs(inputs.StatorCurrent) >= kHighCurrentThreshold && !inputs.LimitSwitch && !Set.of(CLOSED, OPENED, SLOW_CLOSE).contains(currentState)) {
            highCurrentFrames++;
            if (highCurrentFrames >= kHighCurrentThresholdFrames) {
                highCurrentFrames = 0;
                changeStateForce(SAVE_OPEN);
            }
        } else {
            highCurrentFrames = 0;
        }

        io.updateInputs(inputs);
        Logger.processInputs("Intake Rail", inputs);

        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.sequence(
            setPercentCmd(-0.5),
            Commands.waitUntil(this::isReset),
            Commands.waitSeconds(0.06),
            setPositionCmd(PositionsConstants.IntakeRail.kOpen.get()),
            Commands.waitUntil(this::atGoal)
        ));

        addEdge(RESET, OPENED);

        addEdge(OPENED, CLOSED, Commands.sequence(
            setPositionCmd(PositionsConstants.IntakeRail.kClose.get()),
            Commands.waitUntil(this::atGoal)
        ));

        addEdge(List.of(CLOSED, SLOW_CLOSE, SAVE_OPEN), OPENED, () -> Commands.sequence(
            setPositionCmd(PositionsConstants.IntakeRail.kOpen.get()),
            Commands.waitUntil(this::atGoal)
        ));

        addEdge(OPENED, SLOW_CLOSE, Commands.sequence(
//            new LoopCommand(
//                Commands.sequence(
//                    setPercentCmd(0.4),
//                    Commands.waitUntil(() -> getPosition() > PositionsConstants.IntakeRail.kSlowCloseHighThresh.get()),
//                    setPercentCmd(-0.4),
//                    Commands.waitUntil(() -> getPosition() < PositionsConstants.IntakeRail.kSlowCloseLowThresh.get() || isReset())
//                ),
//                3
//            ),
            setPercentCmd(-0.4),
            Commands.waitUntil(() -> getPosition() < PositionsConstants.IntakeRail.kSlowCloseLowThresh.get() || isReset()),
            setPositionCmd(PositionsConstants.IntakeRail.kSlowCloseLowThresh.get())
        ));

        addEdge(List.of(CLOSED, OPENED, SLOW_CLOSE), SAVE_OPEN, () -> Commands.sequence(
            setPercentCmd(0.5)
        ));


        addStateEnd(RESET, () -> true, OPENED);

        addStateEnd(SLOW_CLOSE, () -> true, OPENED);

        addStateEnd(SAVE_OPEN, () -> inputs.LimitSwitches[1], OPENED);
    }

    private boolean isReset() {
        return !enabled || inputs.LimitSwitches[0];
    }

    private boolean atGoal() {
        return !enabled || inputs.AtGoal;
    }

    public void setPercent(double percent) {
        if (!enabled)
            return;

        io.setPercent(percent);
    }

    public Command setPercentCmd(double percent) {
        return Commands.runOnce(() -> setPercent(percent));
    }

    private double getPosition() {
        if (!enabled)
            return 0;

        return inputs.Position;
    }

    private void setPosition(double position) {
        if (!enabled)
            return;

        io.setPosition(position);
    }

    private Command setPositionCmd(double position) {
        return Commands.runOnce(() -> setPosition(position));
    }

    public void stop() {
        if (!enabled)
            return;

        io.stopMotor();
    }

    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }
}