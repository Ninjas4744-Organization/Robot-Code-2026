package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.commands.DetachedCommand;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveController;
import frc.lib.NinjasLib.swerve.SwerveInput;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class SwerveSubsystem extends SubsystemBase {
    private boolean enabled;
    private Command lookHubCommand;

    public SwerveSubsystem(boolean enabled) {
        this.enabled = enabled;

        if (enabled) {
            Swerve.setInstance(new Swerve(SubsystemConstants.kSwerve));
            SwerveController.setInstance(new SwerveController(SubsystemConstants.kSwerveController));
            SwerveController.getInstance().setChannel("Driver");
        }

        lookHubCommand = Commands.sequence(
            Commands.runOnce(() -> {
                SwerveController.getInstance().setChannel("Driver");
            })
        );
    }

    public void swerveDrive(DoubleSupplier leftX, DoubleSupplier leftY, DoubleSupplier rightX) {
        SwerveController.getInstance().setControl(SwerveController.getInstance().fromPercent(
                new SwerveInput(
                        -MathUtil.applyDeadband(leftY.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(leftX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverSpeedFactor,
                        -MathUtil.applyDeadband(rightX.getAsDouble(), GeneralConstants.Swerve.kJoystickDeadband) * GeneralConstants.Swerve.kDriverRotationSpeedFactor,
                        GeneralConstants.Swerve.kDriverFieldRelative
                )), "Driver");
    }

    public Command lookHub() {
        return new DetachedCommand(lookHubCommand);
    }

    public Command lock() {
        return Commands.runOnce(() -> {
            SwerveController.getInstance().setChannel("Lock");
            SwerveController.getInstance().setControl(new SwerveInput(), "Lock");
        });
    }

    public Command close() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            if (DriverStation.isAutonomous()){
                SwerveController.getInstance().setChannel("Auto");
                SwerveController.getInstance().setControl(new SwerveInput(), "Auto");
            }
            else{
                SwerveController.getInstance().setChannel("Driver");
                SwerveController.getInstance().setControl(new SwerveInput(), "Driver");
            }
        });
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.sequence(
            close(),
            Commands.runOnce(() -> Swerve.getInstance().resetModulesToAbsolute())
        );
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        SwerveController.getInstance().periodic();

        Logger.recordOutput("Swerve/Look Hub Command", lookHubCommand.isScheduled() && !lookHubCommand.isFinished());
    }
}