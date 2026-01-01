package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotState;

import java.io.IOException;
import java.util.List;

public class FieldConstants {
    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;
    private static List<Pose2d> reefAprilTags;

    static {
        try {
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);

//                // Switch tag 14 to be 18 for testing. REMOVE BEFORE COMP
//                List<AprilTag> blueTags = kBlueFieldLayout.getTags();
//                blueTags.set(14 - 1, new AprilTag(14, blueTags.get(18 - 1).pose));
//                blueTags.set(9 - 1, new AprilTag(9, blueTags.get(18 - 1).pose));
//                kBlueFieldLayout = new AprilTagFieldLayout(blueTags, kBlueFieldLayout.getFieldLength(), kBlueFieldLayout.getFieldWidth());
//                kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
//
//                List<AprilTag> redTags = kRedFieldLayout.getTags();
//                redTags.set(14 - 1, new AprilTag(14, redTags.get(18 - 1).pose));
//                redTags.set(9 - 1, new AprilTag(9, redTags.get(18 - 1).pose));
//                kRedFieldLayout = new AprilTagFieldLayout(redTags, kRedFieldLayout.getFieldLength(), kRedFieldLayout.getFieldWidth());
//                kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        } catch (IOException e) {
            throw new RuntimeException("Unable to load field layout");
        }

        AprilTagFieldLayout layout = getFieldLayout();
        reefAprilTags = List.of(
            layout.getTagPose(6).get().toPose2d(),
            layout.getTagPose(7).get().toPose2d(),
            layout.getTagPose(8).get().toPose2d(),
            layout.getTagPose(9).get().toPose2d(),
            layout.getTagPose(10).get().toPose2d(),
            layout.getTagPose(11).get().toPose2d(),
            layout.getTagPose(17).get().toPose2d(),
            layout.getTagPose(18).get().toPose2d(),
            layout.getTagPose(19).get().toPose2d(),
            layout.getTagPose(20).get().toPose2d(),
            layout.getTagPose(21).get().toPose2d(),
            layout.getTagPose(22).get().toPose2d()
        );
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

    public static AprilTag nearestReef() {
        AprilTagFieldLayout layout = getFieldLayout();
        List<Integer> reefAprilTagsIds = List.of(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
        Pose2d nearest = RobotState.getInstance().getRobotPose().nearest(reefAprilTags);

        for (int id : reefAprilTagsIds) {
            if (Math.abs(getTagPose(id).toPose2d().getX() - nearest.getX()) < 0.01 && Math.abs(getTagPose(id).toPose2d().getY() - nearest.getY()) < 0.01)
                return new AprilTag(id, getTagPose(id));
        }

        return layout.getTags().get(0);
    }
}
