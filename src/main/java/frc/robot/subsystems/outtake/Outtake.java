package frc.robot.subsystems.outtake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.States;
import org.littletonrobotics.junction.Logger;

public class Outtake extends SubsystemBase {
    private OuttakeIO io;
    private final OuttakeIOInputsAutoLogged inputs = new OuttakeIOInputsAutoLogged();
    private boolean enabled;
    private boolean isCoralInside = false;
    private boolean isAlgaeInside = false;
    private Timer yesAlgaeTimer = new Timer();
    private Timer noAlgaeTimer = new Timer();

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

        if (Math.abs(inputs.Current) > Constants.Outtake.kCurrentThreshold && inputs.Output < 0) {
            if (!yesAlgaeTimer.isRunning())
                yesAlgaeTimer.restart();
        } else {
            yesAlgaeTimer.stop();
            yesAlgaeTimer.reset();
        }

        if (Math.abs(inputs.Current) < Constants.Outtake.kCurrentThreshold) {
            if (!noAlgaeTimer.isRunning())
                noAlgaeTimer.restart();
        } else {
            noAlgaeTimer.stop();
            noAlgaeTimer.reset();
        }

        if(yesAlgaeTimer.get() > 0.25) {
            if (RobotState.getInstance().getRobotState() == States.INTAKE_ALGAE_REEF || RobotState.getInstance().getRobotState() == States.INTAKE_ALGAE_FLOOR)
                isAlgaeInside = true;
        }

        if(noAlgaeTimer.get() > 0.25) {
            isAlgaeInside = false;
        }

        io.periodic();
        io.updateInputs(inputs);
        Logger.processInputs("Outtake", inputs);

        Logger.recordOutput("Outtake/Coral Inside", isCoralInside());
        Logger.recordOutput("Outtake/Algae Inside", isAlgaeInside());
    }

    public Command stop(){
        if (!enabled){
            return Commands.none();
        }
        return Commands.runOnce(() -> io.setPercent(0));
    }

    public Command intake() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(Constants.Outtake.Speeds.Intake.get()));
    }

    public Command intakeAlgae() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> io.setPercent(Constants.Outtake.Speeds.IntakeAlgae.get()));
    }

    public Command outtake() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            io.setPercent(Constants.Outtake.Speeds.Outtake.get());
            isAlgaeInside = false;
            isCoralInside = false;
        });
    }

    public Command outtakeAlgae() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            io.setPercent(Constants.Outtake.Speeds.OuttakeAlgae.get());
            isAlgaeInside = false;
            isCoralInside = false;
        });
    }

    public boolean isCoralInside() {
        if (!enabled)
            return false;

        return isCoralInside;
    }

    public void forceKnowCoralInside(boolean inside) {
        isCoralInside = inside;
    }

    public boolean isAlgaeInside() {
        if (!enabled)
            return false;

        return isAlgaeInside;
    }

//    private boolean hadObjectInside = false;
//    private boolean isReset = false;
    public Command reset() {
        if (!enabled)
            return Commands.none();

        return Commands.runOnce(() -> {
            isCoralInside = false;
            isAlgaeInside = false;
        });
                //Commands.sequence(
//                Commands.runOnce(() -> {
//                    hadObjectInside = false;
//                    isReset = false;
//                }),
//                intake(),
//                Commands.race(
//                        Commands.waitUntil(() -> {
//                            if(currentTimer.get() > 0.125)
//                                hadObjectInside = true;
//                            return hadObjectInside;
//                        }),
//                        Commands.waitSeconds(0.4)
//                ),
//                stop(),
//                Commands.runOnce(() -> {
////                    if (!hadObjectInside) {
////                        isCoralInside = false;
////                        isAlgaeInside = false;
////                    } else if (!isCoralInside && !isAlgaeInside) {
////                        Command outtake = outtake().andThen(Commands.waitSeconds(0.5)).andThen(stop());
////                        outtake.schedule();
////                    }
//                    if(hadObjectInside)
//                        isCoralInside = true;
//                    else {
//                        isCoralInside = false;
//                        isAlgaeInside = false;
//                    }
//                    isReset = true;
//                })
//        );
    }

    public boolean isReset() {
        return true;//isReset;
    }
}