package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

import java.io.IOException;
import java.util.List;

public class FieldConstants {
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try {
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public static AprilTagFieldLayout getFieldLayoutWithIgnored(List<Integer> ignoredTags) {
        AprilTagFieldLayout layout;

        layout = RobotState.getAlliance() == DriverStation.Alliance.Blue
            ? kBlueFieldLayout
            : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) {
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> ignoredTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayoutWithAllowed(List<Integer> allowedTags) {
        AprilTagFieldLayout layout = getFieldLayout();
        if (!allowedTags.isEmpty()) {
            List<AprilTag> tags = layout.getTags();
            tags.removeIf(tag -> !allowedTags.contains(tag.ID));
            layout = new AprilTagFieldLayout(tags, layout.getFieldLength(), layout.getFieldWidth());
        }

        return layout;
    }

    public static AprilTagFieldLayout getFieldLayout() {
        return getFieldLayoutWithIgnored(List.of());
    }

    public static Pose3d getTagPose(int id) {
        return getFieldLayout().getTagPose(id).get();
    }
}
