package frc.robot.subsystems.climberangle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import org.littletonrobotics.junction.Logger;

public class ClimberAngle extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.AngleControlled,
        ISubsystem.Stoppable,
        ISubsystem.GoalOriented<Rotation2d>
{
    private ClimberAngleIO io;
    private final ClimberAngleIOInputsAutoLogged inputs = new ClimberAngleIOInputsAutoLogged();
    private boolean enabled;

    public ClimberAngle(boolean enabled, ClimberAngleIO io) {
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
        Logger.processInputs("Climber Angle", inputs);
    }

    public Command setVelocity(double velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setVelocity(velocity));
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

        return Math.abs(inputs.Output) < 0.05;
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stop();
    }

    @Override
    public Command setAngle(Rotation2d angle) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPosition(angle.getRadians()));
    }

    @Override
    public Rotation2d getAngle() {
        if (!enabled)
            return null;

        return inputs.Position;
    }

    @Override
    public boolean atGoal() {
        if (!enabled)
            return true;

        return inputs.AtGoal;
    }

    @Override
    public Rotation2d getGoal() {
        if (!enabled)
            return Rotation2d.kZero;

        return Rotation2d.fromRotations(inputs.Goal);
    }
}