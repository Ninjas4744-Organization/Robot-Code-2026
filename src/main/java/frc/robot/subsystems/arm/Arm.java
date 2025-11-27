package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
    private boolean enabled;

    public Arm(boolean enabled, ArmIO io) {
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
        Logger.processInputs("Arm", inputs);
    }

    public Command setAngle(Rotation2d angle){
        if (!enabled) {
            return Commands.none();
        }

        return Commands.runOnce(() -> io.setPosition(angle));
    }

//    public Command setAngleSmart(Supplier<Rotation2d> angle) {
//        if (!enabled) {
//            return Commands.none();
//        }
//        return Commands.run(() -> {
//            double elevatorHeight = RobotContainer.getElevator().getHeight();
//            Rotation2d intakeAngle = RobotContainer.getIntakeAngle().getAngle().rotateBy(Rotation2d.fromRadians(1.33));
//
//            double armLength = 0.661302;
//            double baseArmToChassisDist = 0.36339;
//            double intakeLength = 0.355595;
//            double baseIntakeToChassisDist = 0.159569;
//            double chassisHeight = 0;
//            double safeDist = 0.05;
//
//            double targetArmHeight = armLength * angle.get().getSin() + elevatorHeight + baseArmToChassisDist;
//            double intakeHeight = intakeLength * intakeAngle.getSin() + baseIntakeToChassisDist;
//
//            if (targetArmHeight > intakeHeight) {
//                if(targetArmHeight > chassisHeight)
//                    io.setPosition(angle.get());
//                else {
//                    targetArmHeight = chassisHeight + safeDist;
//                    double targetAngle = Math.asin((targetArmHeight - elevatorHeight - baseArmToChassisDist) / armLength);
//                    io.setPosition(Rotation2d.fromRadians(targetAngle));
//                }
//            } else {
//                targetArmHeight = intakeHeight + safeDist;
//                double targetAngle = Math.asin((targetArmHeight - elevatorHeight - baseArmToChassisDist) / armLength);
//                io.setPosition(Rotation2d.fromRadians(targetAngle));
//            }
//        }).until(() -> Math.abs(angle.get().getRotations() - inputs.Position) < Constants.Arm.kControllerConstants.real.positionGoalTolerance);
//    }

    public Rotation2d getAngle() {
        if (!enabled) {
            return Rotation2d.kZero;
        }

        return Rotation2d.fromRotations(inputs.Position);
    }

    public boolean atGoal() {
        if (!enabled){
            return true;
        }

        return inputs.AtGoal;
    }

    public Command reset() {
        if (!enabled){
            return Commands.none();
        }

        return Commands.runOnce(() -> io.setEncoder(inputs.AbsoluteAngle.getRotations()));
    }

    public boolean isReset(){
       if (!enabled){
            return true;
       }

       return inputs.AtGoal;
    }
}