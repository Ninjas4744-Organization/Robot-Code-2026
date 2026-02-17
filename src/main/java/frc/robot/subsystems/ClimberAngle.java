package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.controllers.Controller;
import frc.lib.NinjasLib.controllers.ControllerIOInputsAutoLogged;
import frc.lib.NinjasLib.subsystem.IO;
import frc.lib.NinjasLib.subsystem.ISubsystem;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberAngle extends SubsystemBase implements
    ISubsystem.Resettable,
    ISubsystem.AngleControlled,
    ISubsystem.Stoppable,
    ISubsystem.GoalOriented<Rotation2d>
{
    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    public ClimberAngle(boolean enabled) {
        this.enabled = enabled;
        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kClimberAngle);
            else
                this.io = new IO.All<>(){};
            io.setup();
        }
    }

    @Override
    public void periodic() {
        if (!enabled) return;
        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Climber Angle", inputs);
    }

    @Override
    public void stop() {
        if (!enabled) return;
        io.stopMotor();
    }

    @Override
    public Command stopCmd() {
        return Commands.runOnce(this::stop);
    }

    @Override
    public boolean isReset() {
        return !enabled || inputs.LimitSwitch || GeneralConstants.kRobotMode.isSim();
    }

    @Override
    public Command reset() {
        if (!enabled) return Commands.none();
        return Commands.runOnce(() -> io.setPercent(-0.4))
            .andThen(Commands.waitUntil(this::isReset))
            .finallyDo(this::stop);
    }

    @Override
    public void setAngle(Rotation2d angle) {
        if (!enabled) return;
        io.setPosition(angle.getRotations());
    }

    @Override
    public Command setAngleCmd(Rotation2d angle) {
        return Commands.runOnce(() -> setAngle(angle));
    }

    @Override
    public Rotation2d getAngle() {
        return enabled ? Rotation2d.fromRotations(inputs.Position) : Rotation2d.kZero;
    }

    @Override
    public boolean atGoal() {
        return !enabled || inputs.AtGoal;
    }

    @Override
    public Rotation2d getGoal() {
        return enabled ? Rotation2d.fromRotations(inputs.Goal) : Rotation2d.kZero;
    }
}