package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.States;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class Outtake extends SubsystemBase {
    private OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    private boolean enabled;
    boolean isCoralInside = false;
    boolean isAlgaeInside = false;

    public Outtake(boolean enabled, OuttakeIO io) {
        if (enabled) {
            this.io = io;
            io.setup();
        }
        this.enabled = enabled;
    }

    @Override
    public void periodic() {
        if (!enabled)
            return;

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Outtake", inputs);

        Logger.recordOutput("Outtake/Coral Inside", isCoralInside());
        Logger.recordOutput("Outtake/Algae Inside", isAlgaeInside());
    }

    public Command setPercent(DoubleSupplier percent) {
        if (!enabled){
            return Commands.none();
        }

        return Commands.runOnce(() -> io.setPercent(percent.getAsDouble()));
    }

    public Command stop() {
        if (!enabled){
            return Commands.none();
        }

        return Commands.runOnce(() -> io.setPercent(0));
    }

    public boolean isCoralInside() {
        if (!enabled)
            return false;

        return isCoralInside;
    }

    public void forceKnowCoralInside(boolean inside) {
        isCoralInside = inside;
    }

    public void forceKnowAlgaeInside(boolean inside) {
        isAlgaeInside = inside;
    }


    public boolean isAlgaeInside() {
        if (!enabled)
            return false;

        return isAlgaeInside;
    }

    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            isCoralInside = false;
            isAlgaeInside = false;
        });
    }
}