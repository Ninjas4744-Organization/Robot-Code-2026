package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.NinjasLib.statemachine.RobotStateBase;
import frc.robot.RobotState;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class FieldConstants {
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try {
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2026RebuiltWelded.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public static Optional<AprilTagFieldLayout> getFieldLayoutWithIgnored(List<Integer> ignoredTags) {
        if (RobotStateBase.getAlliance().isEmpty())
            return Optional.empty();

        AprilTagFieldLayout layout;

        layout = RobotStateBase.getAlliance().get() == DriverStation.Alliance.Blue
            ? kBlueFieldLayout
            : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) {
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> ignoredTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return Optional.of(layout);
    }

    public static Optional<AprilTagFieldLayout> getFieldLayoutWithAllowed(List<Integer> allowedTags) {
        if (getFieldLayout().isEmpty())
            return Optional.empty();

        AprilTagFieldLayout layout = getFieldLayout().get();
        if (!allowedTags.isEmpty()) {
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> !allowedTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return Optional.of(layout);
    }

    public static Optional<AprilTagFieldLayout> getFieldLayout() {
        return getFieldLayoutWithIgnored(List.of());
    }

    public static Optional<Pose3d> getTagPose(int id) {
        if (getFieldLayout().isEmpty())
            return Optional.empty();

        return Optional.of(getFieldLayout().get().getTagPose(id).get());
    }

    public static Optional<Pose3d> getHubPose() {
        if (getFieldLayout().isEmpty())
            return Optional.empty();

        return Optional.of(new Pose3d(Units.inchesToMeters(158.6 + 47 / 2.0), getFieldLayout().get().getFieldWidth() / 2, Units.inchesToMeters(72), new Rotation3d()));
    }

    public static Optional<Translation2d> getTranslationToHub() {
        if (getHubPose().isEmpty())
            return Optional.empty();

        return Optional.of(RobotState.get().getTranslation(getHubPose().get().toPose2d()));
    }

    public static Optional<Double> getDistToHub() {
        if (getHubPose().isEmpty())
            return Optional.empty();

        return Optional.of(RobotState.get().getDistance(getHubPose().get().toPose2d()));
    }

    public static Pose2d getLeftTrenchPose() {
        return new Pose2d(4.65, 7.4, Rotation2d.kZero);
    }

    public static Pose2d getRightTrenchPose() {
        return new Pose2d(4.65, 0.6, Rotation2d.kZero);
    }

    public static boolean atAllianceZone() {
        return RobotState.get().getRobotPose().getX() < PositionsConstants.Swerve.kAllianceXThreshold.get();
    }

    public static boolean atNeutralZone() {
        return RobotState.get().getRobotPose().getX() > PositionsConstants.Swerve.kNeutralXThreshold.get();
    }

    public static boolean nearLeftTrench() {
        double leftTrenchDist = RobotState.get().getDistance(FieldConstants.getLeftTrenchPose());
        double robotY = RobotState.get().getRobotPose().getY();
        return leftTrenchDist < PositionsConstants.Swerve.kAutoTrenchThreshold.get() && Math.abs(FieldConstants.getLeftTrenchPose().getY() - robotY) < PositionsConstants.Swerve.kAutoTrenchYThreshold.get();
    }

    public static boolean nearRightTrench() {
        double rightTrenchDist = RobotState.get().getDistance(FieldConstants.getRightTrenchPose());
        double robotY = RobotState.get().getRobotPose().getY();
        return rightTrenchDist < PositionsConstants.Swerve.kAutoTrenchThreshold.get() && Math.abs(FieldConstants.getRightTrenchPose().getY() - robotY) < PositionsConstants.Swerve.kAutoTrenchYThreshold.get();
    }
}
