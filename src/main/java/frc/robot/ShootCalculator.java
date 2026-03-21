package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.lib.NinjasLib.swerve.SwerveSpeeds;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.PositionsConstants;
import org.littletonrobotics.junction.Logger;

public class ShootCalculator extends SubsystemBase {
    private static final double iterations = 5;
    private static final double predictSec = 0;
    private static ShootParams shootParams;

    public record ShootParams(Rotation2d angle, double virtualDist, Translation2d virtualTarget) {}
    public record RobotPrediction(Pose2d pose, SwerveSpeeds speeds) {}

    public static RobotPrediction predict(Pose2d currentPose, SwerveSpeeds currentSpeeds, SwerveSpeeds accel, double dt) {
        // 1. Predict Future Velocity (v = u + at)
        SwerveSpeeds futureSpeeds = new SwerveSpeeds(currentSpeeds.plus(accel.times(dt)), currentSpeeds.fieldRelative);

        // 2. Calculate Average Velocity for Pose displacement
        // Using the average (v_initial + v_final) / 2 is more accurate than just v_initial
        double avgVx = (currentSpeeds.vxMetersPerSecond + futureSpeeds.vxMetersPerSecond) / 2.0;
        double avgVy = (currentSpeeds.vyMetersPerSecond + futureSpeeds.vyMetersPerSecond) / 2.0;
        double avgOmega = (currentSpeeds.omegaRadiansPerSecond + futureSpeeds.omegaRadiansPerSecond) / 2.0;

        // 3. Create a Twist2d for the curved path
        // Twist represents the change in x, y, and theta in the robot's local frame
        Twist2d twist = new Twist2d(
            avgVx * dt,
            avgVy * dt,
            avgOmega * dt
        );

        // 4. Apply the twist to the current pose using Exponential Mapping
        Pose2d futurePose = currentPose.exp(twist);

        return new RobotPrediction(futurePose, futureSpeeds);
    }

    private static Translation2d calculateLookaheadTarget(Translation2d originalTarget, Pose2d robotPose, SwerveSpeeds speeds) {
        Translation2d robotVel = speeds.toTranslation();
        Translation2d target = new Translation2d(originalTarget.getX(), originalTarget.getY());

        for (int i = 0; i < iterations; i++) {
            double distTarget = robotPose.getTranslation().getDistance(target);
            double airTime = PositionsConstants.Shooter.getAirTime(distTarget);
            target = originalTarget.minus(robotVel.times(airTime));
        }

        return target;
    }

    public static ShootParams calculateShootParams(Translation2d target) {
        Translation2d virtualTarget;
        if (predictSec > 0) {
            RobotPrediction prediction = predict(RobotState.get().getRobotPose(),
                Swerve.getInstance().getSpeeds().getAsFieldRelative(),
                new SwerveSpeeds(RobotContainer.getSwerve().getAcceleration(), 0, true),
                predictSec);

            virtualTarget = calculateLookaheadTarget(target, RobotState.get().getRobotPose(), prediction.speeds);
        } else {
            virtualTarget = calculateLookaheadTarget(target, RobotState.get().getRobotPose(), Swerve.getInstance().getSpeeds().getAsFieldRelative());
        }

        Translation2d robotPose = RobotState.get().getRobotPose().getTranslation();
        shootParams = new ShootParams(virtualTarget.minus(robotPose).getAngle(), robotPose.getDistance(virtualTarget), virtualTarget);

        return shootParams;
    }

    public static ShootParams getShootParams() {
        return shootParams;
    }

    @Override
    public void periodic() {
        if (RobotState.getShootingMode() == ShootingMode.DELIVERY)
            calculateShootParams(PositionsConstants.Swerve.Delivery.getDeliveryTarget().getTranslation());
        else
            calculateShootParams(FieldConstants.getHubPose().get().toPose2d().getTranslation());

        Logger.recordOutput("ShootCalculator/Virtual Target", new Pose2d(getShootParams().virtualTarget, Rotation2d.kZero));
        Logger.recordOutput("ShootCalculator/Shooting Ready", RobotState.isShootReady());
        Logger.recordOutput("ShootCalculator/Distance Hub", FieldConstants.getDistToHub().get());
        Logger.recordOutput("ShootCalculator/Distance Virtual Target", getShootParams().virtualDist);
    }
}
