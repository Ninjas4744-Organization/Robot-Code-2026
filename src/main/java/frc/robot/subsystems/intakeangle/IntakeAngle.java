package frc.robot.subsystems.intakeangle;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

public class IntakeAngle extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.AngleControlled,
        ISubsystem.GoalOriented<Rotation2d>
{
    private IntakeAngleIO io;
    private final IntakeAngleIOInputsAutoLogged inputs = new IntakeAngleIOInputsAutoLogged();
    private boolean enabled;

    public IntakeAngle(boolean enabled, IntakeAngleIO io) {
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
        Logger.processInputs("Intake Angle", inputs);
    }

    @Override
    public Command setAngle(Rotation2d angle) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(
            () -> io.setPosition(angle.getRadians())
        );
    }

    @Override
    public Rotation2d getAngle(){
        if (!enabled) {
            return Rotation2d.kZero;
        }
        return Rotation2d.fromRadians(inputs.Position);
    }

    @Override
    public boolean atGoal(){
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

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            io.setEncoder(inputs.AbsoluteAngle.getRotations());
            io.setPosition(PositionsConstants.IntakeAngle.kClose.get() / 360);
        });
    }

    public boolean isReset() {
        if (!enabled)
            return true;

        return inputs.AtGoal;
    }
}