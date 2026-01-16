package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import frc.robot.RobotState;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.PositionControlled,
        ISubsystem.PercentControlled,
        ISubsystem.GoalOriented<Double>,
        ISubsystem.Stoppable
{
    private ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
    private boolean enabled;

    public Climber(boolean enabled, ClimberIO io) {
        this.enabled = enabled;

        if (enabled) {
            this.io = io;
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    @Override
    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(io::stopMotor);
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return inputs.LimitSwitch;
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(-0.4)).andThen(Commands.waitUntil(() -> inputs.LimitSwitch)).finallyDo(() -> io.setPercent(0));
    }

    @Override
    public Command setPosition(double position) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPosition(position));
    }

    @Override
    public double getPosition() {
        if (!enabled)
            return 0;

        return inputs.Position;
    }

    @Override
    public boolean atGoal() {
        if (!enabled)
            return false;

        return inputs.AtGoal;
    }

    @Override
    public Double getGoal() {
        if (!enabled)
            return 0.0;

        return inputs.Goal;
    }

    @Override
    public Command setPercent(double percent) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(percent));
    }

    @Override
    public double getOutput() {
        if (!enabled)
            return 0;

        return inputs.Output;
    }
}