package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.BackgroundCommand;
import frc.lib.NinjasLib.subsystem_interfaces.ISubsystem;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements
        ISubsystem.Resettable,
        ISubsystem.VelocityControlled,
        ISubsystem.GoalOriented<Double>,
        ISubsystem.Stoppable
{
    private ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
    private boolean enabled;
    private BackgroundCommand backgroundCommand;

    public Shooter(boolean enabled, ShooterIO io) {
        this.enabled = enabled;

        if (enabled) {
            this.io = io;
            io.setup();

            backgroundCommand = new BackgroundCommand();
        }
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter Command", backgroundCommand.isRunning());
    }

    @Override
    public Command setVelocity(double velocity) {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setVelocity(velocity));
    }

    @Override
    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
    }

    @Override
    public boolean atGoal(){
        if (!enabled)
            return true;

        return inputs.AtGoal;
    }

    @Override
    public Double getGoal() {
        if (!enabled)
            return 0.0;

        return inputs.Goal;
    }

    @Override
    public Command stop() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            backgroundCommand.stop();
            io.stopMotor();
        });
    }

    @Override
    public boolean isReset() {
        if (!enabled)
            return true;

        return Math.abs(inputs.Velocity) < 5 && !backgroundCommand.isRunning();
    }

    @Override
    public Command reset() {
        if (!enabled)
            return Commands.none();

        return stop();
    }

    public Command autoHubVelocity() {
        if (!enabled)
            return Commands.none();

        return backgroundCommand.setNewTaskCommand(Commands.run(() -> {
            double relativeXVel = Swerve.getInstance().getSpeeds().vxMetersPerSecond;
            double relativeXWantedVel = Swerve.getInstance().getWantedSpeeds().vxMetersPerSecond;

            double shootFix = (PositionsConstants.Shooter.getShootFix(Math.abs(relativeXWantedVel)) * -Math.signum(relativeXWantedVel)) * 0.8
                    + (PositionsConstants.Shooter.getShootFix(Math.abs(relativeXVel)) * -Math.signum(relativeXVel)) * 0.2;

            io.setVelocity(PositionsConstants.Shooter.getShootSpeed(FieldConstants.getDistToHub()) + shootFix);

            Logger.recordOutput("Shoot Fix", shootFix);
        }));
    }

    public Command autoDeliveryVelocity() {
        if (!enabled)
            return Commands.none();

        return backgroundCommand.setNewTaskCommand(Commands.run(() -> {
            double dist = RobotState.getInstance().getDistance(PositionsConstants.Swerve.getDeliveryTarget());
            io.setVelocity(PositionsConstants.Shooter.getDeliverySpeed(dist));
        }));
    }
}