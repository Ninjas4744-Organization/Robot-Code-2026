package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.NinjasLib.localization.vision.Vision;
import frc.lib.NinjasLib.localization.vision.VisionOutput;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.RobotState;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
    private double odometryDrift = 0;
    private Pose2d lastVisionPose = new Pose2d();

    public VisionSubsystem() {
        Vision.setInstance(new Vision(SubsystemConstants.kVision));
    }

    @Override
    public void periodic() {
        Vision.getInstance().periodic();

        Logger.recordOutput("Vision/Odometry Drift", odometryDrift);
        odometryDrift += Swerve.getInstance().getOdometryTwist().getNorm() * GeneralConstants.Vision.kOdometryDriftPerMeter;

        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
        for (VisionOutput estimation : estimations) {
            if(!estimation.hasTargets)
                continue;

            Logger.recordOutput("Vision/" + estimation.cameraName + "/Last Vision Pose", estimation.robotPose);

            Matrix<N3, N1> strength = getVisionStrength(estimation);

            ChassisSpeeds speed = Swerve.getInstance().getChassisSpeeds(false);
            boolean passedFilters = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) <= GeneralConstants.Vision.kMaxSpeedFilter
                && speed.omegaRadiansPerSecond <= GeneralConstants.Vision.kMaxAngularSpeedFilter
                && estimation.closestTargetDist <= GeneralConstants.Vision.kMaxDistanceFilter
                && estimation.closestTargetDist >= GeneralConstants.Vision.kMinDistanceFilter
                && estimation.ambiguity <= GeneralConstants.Vision.kMaxAmbiguityFilter;

            Logger.recordOutput("Vision/" + estimation.cameraName + "/Passed Filters", passedFilters);

            if (passedFilters || DriverStation.isDisabled()) {
//                RobotState.getInstance().updateRobotPose(estimation.robotPose, estimation.timestamp, strength);
                RobotState.getInstance().updateRobotPose(estimation, VecBuilder.fill(1, 1, 1));
                lastVisionPose = estimation.robotPose;

                odometryDrift *= 1 - strength.get(0, 0);
            }
        }
    }

    public Matrix<N3, N1> getVisionStrength(VisionOutput estimation) {
        double a = 4;
        double visionStrength = 1 / (1 + a * Math.pow(odometryDrift, -0.5) * estimation.closestTargetDist * estimation.closestTargetDist);

        return VecBuilder.fill(visionStrength, visionStrength, visionStrength / 2);
    }

    public double getOdometryDrift() {
        return odometryDrift;
    }

    public Pose2d getLastVisionPose() {
        return lastVisionPose;
    }
}
