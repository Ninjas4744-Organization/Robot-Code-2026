package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private boolean enabled = true;

    private double odometryDrift = 0;
    private Pose2d lastVisionPosePassed = new Pose2d();
    private Pose2d lastVisionPose = new Pose2d();

    private Pose2d megaTag1Pose = new Pose2d();
    private double megaTag1DistFromTag;

    private boolean resettedGyro = false;
    private boolean resettedPose = false;
    private int framesSinceGyroUpdate = 75;

    public VisionSubsystem() {
        Vision.setInstance(new Vision(SubsystemConstants.kVision));

        if (GeneralConstants.kRobotMode.isSim()) {
            resettedGyro = true;
            resettedPose = true;
        }
    }

    @Override
    public void periodic() {
        Vision.getInstance().periodic();

        odometryDrift += Swerve.getInstance().getOdometryTwist().getNorm() * GeneralConstants.Vision.kOdometryDriftPerMeter;
        Logger.recordOutput("Vision/Odometry Drift", odometryDrift);

        if (DriverStation.isDisabled() && !GeneralConstants.kRobotMode.isSim()) {
            framesSinceGyroUpdate++;
            if (framesSinceGyroUpdate >= 75/* && getMegaTag1Pose() != null*/) {
//                RobotState.get().resetGyro(getMegaTag1Pose().getRotation());
                RobotState.get().resetGyro(Rotation2d.k180deg);
                resettedGyro = true;
                framesSinceGyroUpdate = 0;
            }
        }

        megaTag1Pose = null;
        megaTag1DistFromTag = 0;
        VisionOutput[] estimations = Vision.getInstance().getVisionEstimations();
        for (VisionOutput estimation : estimations) {
            if(!estimation.hasTargets)
                continue;

            Matrix<N3, N1> strength = getVisionStrength(estimation);
            if (DriverStation.isDisabled())
                strength = VecBuilder.fill(1, 1, 1);
            boolean passedFilters = isPassedFilters(estimation);

            megaTag1Pose = estimation.robotPoseMegaTag1;
            megaTag1DistFromTag = estimation.closestTargetDistMegaTag1;
            lastVisionPose = estimation.robotPose;

            Logger.recordOutput("Vision/" + estimation.cameraName + "/Vision Pose", estimation.robotPose);
            Logger.recordOutput("Vision/" + estimation.cameraName + "/Passed Filters", passedFilters);
            Logger.recordOutput("Vision/" + estimation.cameraName + "/Strength", strength.get(0, 0));

            if ((passedFilters || (DriverStation.isDisabled() && resettedGyro)) && enabled) {
                Logger.recordOutput("Vision/" + estimation.cameraName + "/Vision Pose (Passed Filters)", estimation.robotPose);

                RobotState.get().updateRobotPose(estimation.robotPose, estimation.timestamp, strength);
                lastVisionPosePassed = estimation.robotPose;

                odometryDrift *= 1 - strength.get(0, 0);
                resettedPose = true;
            }
        }

        Logger.recordOutput("Vision/MegaTag 1 Vision", getMegaTag1Pose());
        Logger.recordOutput("Vision/MegaTag 1 Vision Dist", getMegaTag1DistFromTag());
        Logger.recordOutput("Vision/Odometry Only Pose", RobotState.get().getOdometryOnlyRobotPose());
        Logger.recordOutput("Vision/Odometry Vision Error", getLastVisionPose().getTranslation().getDistance(RobotState.get().getOdometryOnlyRobotPose().getTranslation()));
    }

    public Matrix<N3, N1> getVisionStrength(VisionOutput estimation) {
        double a = 1.5;
        double epsilon = 0.05;
        double visionStrength = 1 / (1 + 1 / a / Math.max(Math.pow(odometryDrift + epsilon, 0.5), Math.pow(odometryDrift + epsilon, 2)) * Math.pow(estimation.closestTargetDist, 2));

        return VecBuilder.fill(visionStrength, visionStrength, visionStrength / 2);
    }

    private boolean isPassedFilters(VisionOutput estimation) {
        ChassisSpeeds speed = Swerve.getInstance().getSpeeds();

        return Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond) <= GeneralConstants.Vision.kMaxSpeedFilter
            && speed.omegaRadiansPerSecond <= GeneralConstants.Vision.kMaxAngularSpeedFilter
            && estimation.closestTargetDist <= GeneralConstants.Vision.kMaxDistanceFilter
            && estimation.closestTargetDist >= GeneralConstants.Vision.kMinDistanceFilter
            && estimation.ambiguity <= GeneralConstants.Vision.kMaxAmbiguityFilter;
    }

    public double getOdometryDrift() {
        return odometryDrift;
    }

    public Pose2d getLastVisionPosePassed() {
        return lastVisionPosePassed;
    }

    public Pose2d getLastVisionPose() {
        return lastVisionPose;
    }

    public Pose2d getMegaTag1Pose() {
        return megaTag1Pose;
    }

    public double getMegaTag1DistFromTag() {
        return megaTag1DistFromTag;
    }

    public boolean isResettedGyro() {
        return resettedGyro;
    }

    public boolean isResettedPose() {
        return resettedPose;
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }
}
