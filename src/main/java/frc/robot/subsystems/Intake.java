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

import static frc.robot.subsystems.Intake.IntakeStates.*;

public class Intake extends StateMachineBase<Intake.IntakeStates> {
    public enum IntakeStates {
        IDLE,
        INTAKE,
        SAVE_OUTTAKE
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

    private int highCurrentFrames = 0;
    private static final int kHighCurrentThreshold = 75;
    private static final int kHighCurrentThresholdFrames = 6;

    public Intake(boolean enabled) {
        super(IntakeStates.class);
        currentState = IDLE;

        this.enabled = enabled;

        if (enabled) {
            if (!GeneralConstants.kRobotMode.isReplay())
                this.io = new IO.BasicIOController(Controller.ControllerType.TalonFX, SubsystemConstants.kIntake);
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

        if (Math.abs(inputs.StatorCurrent) >= kHighCurrentThreshold && currentState == INTAKE) {
            highCurrentFrames++;
            if (highCurrentFrames >= kHighCurrentThresholdFrames) {
                highCurrentFrames = 0;
                changeStateForce(SAVE_OUTTAKE);
            }
        } else {
            highCurrentFrames = 0;
        }

        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        super.periodic();
    }

    @Override
    protected void define() {
        addEdge(List.of(IDLE, SAVE_OUTTAKE), INTAKE, () -> setVelocityCmd(PositionsConstants.Intake.kIntake.get()));

        addEdge(INTAKE, IDLE, stopCmd());

        addEdge(INTAKE, SAVE_OUTTAKE, setVelocityCmd(PositionsConstants.Intake.kOuttake.get()));

        addStateEnd(SAVE_OUTTAKE, Commands.waitSeconds(0.1), INTAKE);
    }

    public void setVelocity(double velocity) {
        if (!enabled)
            return;

        io.setVelocity(velocity);
    }

    public Command setVelocityCmd(double velocity) {
        return Commands.runOnce(() -> setVelocity(velocity));
    }

    public double getVelocity() {
        if (!enabled)
            return 0;

        return inputs.Velocity;
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