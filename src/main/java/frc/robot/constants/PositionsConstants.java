package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.lib.NinjasLib.LoggedTunableNumber;

public class PositionsConstants {
    public static class ClimberAngle {
        public static final LoggedTunableNumber kClose = new LoggedTunableNumber("Climber Angle/Close", 90, false);
        public static final LoggedTunableNumber kOpen = new LoggedTunableNumber("Climber Angle/Open", 0, false);
    }

    public static class Climber {
        // I considered LeftClimb is the starting pos, so the system starts with the left hook down
        public static final LoggedTunableNumber kLeftClimb = new LoggedTunableNumber("Climber/Left Climb", -90, false);
        public static final LoggedTunableNumber kRightClimb = new LoggedTunableNumber("Climber/Right Climb", 90, false);
        public static final LoggedTunableNumber kClimbReady = new LoggedTunableNumber("Climber/Left Climb", -45, false);
        // half climb for auto, so the second hook wouldn't lock and the robot could go down
        public static final LoggedTunableNumber kRightAutoClimb = new LoggedTunableNumber("Climber/Right Climb", -60, false);
    }

    public static class Intake {
        public static final LoggedTunableNumber kIntake = new LoggedTunableNumber("Intake/Intake", 0.35, false);
    }

    public static class Indexer {
        public static final LoggedTunableNumber kIndex = new LoggedTunableNumber("Indexer/Index", 0.35, false);
    }

    public static class Indexer2 {
        public static final LoggedTunableNumber kIndex = new LoggedTunableNumber("Indexer2/Index", 100, false);
    }

    public static class IntakeAngle {
        public static final LoggedTunableNumber kClose = new LoggedTunableNumber("Intake Angle/Close", 90, false);
        public static final LoggedTunableNumber kOpen = new LoggedTunableNumber("Intake Angle/Open", -90, false);
    }

    public static class Shooter {
        public static final LoggedTunableNumber kShoot = new LoggedTunableNumber("Shooter/Shoot", 100, false);
        public static final LoggedTunableNumber kDelivery = new LoggedTunableNumber("Shooter/Delivery", 80, false);
        public static final LoggedTunableNumber kDump = new LoggedTunableNumber("Shooter/Dump", 50, false);

        private static final InterpolatingDoubleTreeMap kShootMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kShoot0 = new LoggedTunableNumber("Shooter/Shoot/0", 47, true);
        private static final LoggedTunableNumber kShoot2 = new LoggedTunableNumber("Shooter/Shoot/2", 49, true);
        private static final LoggedTunableNumber kShoot25 = new LoggedTunableNumber("Shooter/Shoot/2.5", 52, true);
        private static final LoggedTunableNumber kShoot3 = new LoggedTunableNumber("Shooter/Shoot/3", 55, true);
        private static final LoggedTunableNumber kShoot35 = new LoggedTunableNumber("Shooter/Shoot/3.5", 60, true);
        private static final LoggedTunableNumber kShoot4 = new LoggedTunableNumber("Shooter/Shoot/4", 64, true);
        private static final LoggedTunableNumber kShoot45 = new LoggedTunableNumber("Shooter/Shoot/4.5", 68, true);
        private static final LoggedTunableNumber kShoot5 = new LoggedTunableNumber("Shooter/Shoot/5", 77, true);

        public static double getShootSpeed(double dist) {
            kShootMap.put(0.0, kShoot0.get());
            kShootMap.put(2.0, kShoot2.get());
            kShootMap.put(2.5, kShoot25.get());
            kShootMap.put(3.0, kShoot3.get());
            kShootMap.put(3.5, kShoot35.get());
            kShootMap.put(4.0, kShoot4.get());
            kShootMap.put(4.5, kShoot45.get());
            kShootMap.put(5.0, kShoot5.get());

            return kShootMap.get(dist);
        }

        private static final InterpolatingDoubleTreeMap kShootFixMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kShootFix0 = new LoggedTunableNumber( "Shooter/ShootFix/0",   0, true);
        private static final LoggedTunableNumber kShootFix05 = new LoggedTunableNumber("Shooter/ShootFix/0.5", 1, true);
        private static final LoggedTunableNumber kShootFix1 = new LoggedTunableNumber( "Shooter/ShootFix/1",   6, true);
        private static final LoggedTunableNumber kShootFix15 = new LoggedTunableNumber("Shooter/ShootFix/1.5", 11, true);
        private static final LoggedTunableNumber kShootFix2 = new LoggedTunableNumber( "Shooter/ShootFix/2",   16, true);
        private static final LoggedTunableNumber kShootFix25 = new LoggedTunableNumber("Shooter/ShootFix/2.5", 21, true);
        private static final LoggedTunableNumber kShootFix3 = new LoggedTunableNumber( "Shooter/ShootFix/3",   26, true);
        private static final LoggedTunableNumber kShootFix35 = new LoggedTunableNumber("Shooter/ShootFix/3.5", 31, true);
        private static final LoggedTunableNumber kShootFix4 = new LoggedTunableNumber( "Shooter/ShootFix/4",   36, true);

        public static double getShootFix(double relativeVelocity) {
            kShootFixMap.put(0.0, kShootFix0.get());
            kShootFixMap.put(0.5, kShootFix05.get());
            kShootFixMap.put(1.0, kShootFix1.get());
            kShootFixMap.put(1.5, kShootFix15.get());
            kShootFixMap.put(2.0, kShootFix2.get());
            kShootFixMap.put(2.5, kShootFix25.get());
            kShootFixMap.put(3.0, kShootFix3.get());
            kShootFixMap.put(3.5, kShootFix35.get());
            kShootFixMap.put(4.0, kShootFix4.get());

            return kShootFixMap.get(relativeVelocity);
        }

        private static final InterpolatingDoubleTreeMap kDeliveryMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kDelivery0 = new LoggedTunableNumber("Shooter/Delivery/0", 60, false);
        private static final LoggedTunableNumber kDelivery05 = new LoggedTunableNumber("Shooter/Delivery/0.5", 65, false);
        private static final LoggedTunableNumber kDelivery1 = new LoggedTunableNumber("Shooter/Delivery/1", 70, false);
        private static final LoggedTunableNumber kDelivery15 = new LoggedTunableNumber("Shooter/Delivery/1.5", 75, false);
        private static final LoggedTunableNumber kDelivery2 = new LoggedTunableNumber("Shooter/Delivery/2", 80, false);
        private static final LoggedTunableNumber kDelivery25 = new LoggedTunableNumber("Shooter/Delivery/2.5", 85, false);
        private static final LoggedTunableNumber kDelivery3 = new LoggedTunableNumber("Shooter/Delivery/3", 90, false);
        private static final LoggedTunableNumber kDelivery35 = new LoggedTunableNumber("Shooter/Delivery/3.5", 95, false);
        private static final LoggedTunableNumber kDelivery4 = new LoggedTunableNumber("Shooter/Delivery/4", 100, false);

        public static double getDeliverySpeed(double dist) {
            kDeliveryMap.put(0.0, kDelivery0.get());
            kDeliveryMap.put(0.5, kDelivery05.get());
            kDeliveryMap.put(1.0, kDelivery1.get());
            kDeliveryMap.put(1.5, kDelivery15.get());
            kDeliveryMap.put(2.0, kDelivery2.get());
            kDeliveryMap.put(2.5, kDelivery25.get());
            kDeliveryMap.put(3.0, kDelivery3.get());
            kDeliveryMap.put(3.5, kDelivery35.get());
            kDeliveryMap.put(4.0, kDelivery4.get());

            return kDeliveryMap.get(dist);
        }
    }

    public static class Accelerator {
        public static final LoggedTunableNumber kAccelerate = new LoggedTunableNumber("Accelerator/Accelerate", 100, false);
    }

    public static class Swerve {
        public static final LoggedTunableNumber kHubPositionThreshold = new LoggedTunableNumber("Swerve/Pos Thresh", 0.03, false);
        public static final LoggedTunableNumber kHubAngleThreshold = new LoggedTunableNumber("Swerve/Angle Thresh", 2, false);

        public static final LoggedTunableNumber kHubMinDist = new LoggedTunableNumber("Swerve/Hub Min Dist", 1.5, false);
        public static final LoggedTunableNumber kHubMaxDist = new LoggedTunableNumber("Swerve/Hub Max Dist", 3, false);

        private static final LoggedTunableNumber kDeliveryTargetX = new LoggedTunableNumber("Swerve/Delivery Target X", 3, false);
        private static final LoggedTunableNumber kDeliveryTargetY = new LoggedTunableNumber("Swerve/Delivery Target Y", 3, false);

        public static Pose2d getDeliveryTarget() {
            return new Pose2d(kDeliveryTargetX.get(), kDeliveryTargetY.get(), Rotation2d.kZero);
        }

        private static final InterpolatingDoubleTreeMap kAngleFixMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kAngleFix0 = new LoggedTunableNumber ("Swerve/AngleFix/0",   0, true);
        private static final LoggedTunableNumber kAngleFix05 = new LoggedTunableNumber("Swerve/AngleFix/0.5", 10, true);
        private static final LoggedTunableNumber kAngleFix1 = new LoggedTunableNumber ("Swerve/AngleFix/1",   20, true);
        private static final LoggedTunableNumber kAngleFix15 = new LoggedTunableNumber("Swerve/AngleFix/1.5", 30, true);
        private static final LoggedTunableNumber kAngleFix2 = new LoggedTunableNumber ("Swerve/AngleFix/2",   40, true);
        private static final LoggedTunableNumber kAngleFix25 = new LoggedTunableNumber("Swerve/AngleFix/2.5", 50, true);
        private static final LoggedTunableNumber kAngleFix3 = new LoggedTunableNumber ("Swerve/AngleFix/3",   60, true);
        private static final LoggedTunableNumber kAngleFix35 = new LoggedTunableNumber("Swerve/AngleFix/3.5", 70, true);
        private static final LoggedTunableNumber kAngleFix4 = new LoggedTunableNumber ("Swerve/AngleFix/4",   80, true);

        public static double getAngleFix(double relativeVelocity) {
            kAngleFixMap.put(0.0, kAngleFix0.get());
            kAngleFixMap.put(0.5, kAngleFix05.get());
            kAngleFixMap.put(1.0, kAngleFix1.get());
            kAngleFixMap.put(1.5, kAngleFix15.get());
            kAngleFixMap.put(2.0, kAngleFix2.get());
            kAngleFixMap.put(2.5, kAngleFix25.get());
            kAngleFixMap.put(3.0, kAngleFix3.get());
            kAngleFixMap.put(3.5, kAngleFix35.get());
            kAngleFixMap.put(4.0, kAngleFix4.get());

            return kAngleFixMap.get(relativeVelocity);
        }
    }
}
