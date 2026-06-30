package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.commands.LoopCommand;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.statemachine.StateMachineBase;
import frc.lib.NinjasLib.subsystem.IO;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.PositionsConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.List;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.IntakeRail.IntakeRailState.*;

public class IntakeRail extends StateMachineBase<IntakeRail.IntakeRailState> {
    public enum IntakeRailState {
        UNKNOWN,
        RESET,
        CLOSED,
        OPENED,
        AUTO_OPENED,
        HARD_PUMPING,
        SOFT_PUMPING,
        SAVE_OPEN,
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

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

        io.updateInputs(inputs);
        Logger.processInputs("Intake Rail", inputs);

        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.sequence(
            setPercentCmd(-0.25),
            Commands.waitUntil(this::isReset),
            Commands.waitSeconds(0.2),
            setPositionCmd(PositionsConstants.IntakeRail.kOpen.get()),
            Commands.waitUntil(this::atGoalUp)
        ));

        addEdge(RESET, OPENED);

        addEdge(OPENED, CLOSED, Commands.sequence(
            setPositionCmd(PositionsConstants.IntakeRail.kClose.get()),
            Commands.waitUntil(this::atGoalDown)
        ));

        addEdge(List.of(CLOSED, HARD_PUMPING, SOFT_PUMPING, SAVE_OPEN), OPENED, () -> Commands.sequence(
            setPositionCmd(PositionsConstants.IntakeRail.kOpen.get()),
            Commands.waitUntil(this::atGoalUp)
        ));

        addEdge(CLOSED, AUTO_OPENED, Commands.sequence(
            Commands.runOnce(() -> io.resetVirtualLimits()),
            Commands.waitSeconds(0.5),
            setPositionCmd(PositionsConstants.IntakeRail.kOpen.get()),
            Commands.waitUntil(this::atGoalUp)
        ));
        addEdge(AUTO_OPENED, OPENED);



        addEdge(List.of(OPENED, SOFT_PUMPING), HARD_PUMPING, () -> Commands.sequence(
            new LoopCommand(
                Commands.sequence(
                    setPercentCmd(0.25),
                    Commands.waitUntil(() -> getPosition() > PositionsConstants.IntakeRail.kHardPumpHighThresh.get())
                        .withTimeout(0.7),
                    setPercentCmd(-0.25),
                    Commands.waitUntil(() -> getPosition() < PositionsConstants.IntakeRail.kHardPumpLowThresh.get() || isReset())
                        .withTimeout(0.7)
                ),
                3
            ),
            setPositionCmd(PositionsConstants.IntakeRail.kHardPumpLowThresh.get())
        ));

        addEdge(List.of(OPENED, HARD_PUMPING), SOFT_PUMPING);

        addStateCommand(SOFT_PUMPING, Commands.repeatingSequence(
            setPercentCmd(0.15),
            Commands.waitUntil(() -> getPosition() > PositionsConstants.IntakeRail.kSoftPumpHighThresh.get())
                .withTimeout(0.7),
            setPercentCmd(-0.15),
            Commands.waitUntil(() -> getPosition() < PositionsConstants.IntakeRail.kSoftPumpLowThresh.get() || isReset())
                .withTimeout(0.7)
        ));

        addEdge(List.of(CLOSED, OPENED, HARD_PUMPING, SOFT_PUMPING, RESET), SAVE_OPEN, () -> Commands.sequence(
            setPercentCmd(0.25)
        ));



        addStateEnd(RESET, () -> true, OPENED);

        addStateEnd(HARD_PUMPING, () -> DriverStation.isTeleopEnabled(), SOFT_PUMPING);

        addStateEnd(SAVE_OPEN, Seconds.of(0.5), RESET);

        addStateEnd(AUTO_OPENED, () -> true, OPENED);
    }

    private boolean isReset() {
        return !enabled || inputs.LimitSwitches[0];
    }

    private boolean atGoalUp() {
        return !enabled || inputs.AtGoal || inputs.LimitSwitches[1];
    }

    private boolean atGoalDown() {
        return !enabled || inputs.AtGoal || inputs.LimitSwitches[0];
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