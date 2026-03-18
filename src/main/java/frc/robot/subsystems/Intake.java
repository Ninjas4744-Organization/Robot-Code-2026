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

import static frc.robot.subsystems.Intake.IntakeStates.*;

public class Intake extends StateMachineBase<Intake.IntakeStates> {
    public enum IntakeStates {
        RESET,
        IDLE,
        INTAKE,
    }

    private IO.All<ControllerIOInputsAutoLogged> io;
    private final ControllerIOInputsAutoLogged inputs = new ControllerIOInputsAutoLogged();
    private boolean enabled;

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
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        super.periodic();
    }

    @Override
    protected void define() {
        addOmniEdge(RESET, () -> Commands.runOnce(() -> {
//            setVelocity(PositionsConstants.Intake.kIntake.get()); // TEMP: uncomment
        }));

        addEdge(RESET, INTAKE);
        addEdge(RESET, IDLE); // TEMP: remove

        addEdge(IDLE, INTAKE, setVelocityCmd(PositionsConstants.Intake.kIntake.get()));

        addEdge(INTAKE, IDLE, stopCmd());

        addStateEnd(RESET, () -> true, IDLE); // TEMP: INTAKE
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