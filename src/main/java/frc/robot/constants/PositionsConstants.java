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
        public static final LoggedTunableNumber kIntake = new LoggedTunableNumber("Intake/Intake", 35, false);
    }

    public static class Indexer {
        public static final LoggedTunableNumber kIndex = new LoggedTunableNumber("Indexer/Index", 30, false);
        public static final LoggedTunableNumber kIndexBack = new LoggedTunableNumber("Indexer/Index", -10, false);
    }

    public static class Indexer2 {
        public static final LoggedTunableNumber kIndex = new LoggedTunableNumber("Indexer2/Index", 30, false);
        public static final LoggedTunableNumber kIndexBack = new LoggedTunableNumber("Indexer2/Index", -10, false);
    }

    public static class IntakeOpen {
        public static final LoggedTunableNumber kClose = new LoggedTunableNumber("Intake Open/Close", 0, false);
        public static final LoggedTunableNumber kOpen = new LoggedTunableNumber("Intake Open/Open", 0.5, false);
    }

    public static class Shooter {
        public static final LoggedTunableNumber kShoot = new LoggedTunableNumber("Shooter/Shoot", 100, false);
        public static final LoggedTunableNumber kDelivery = new LoggedTunableNumber("Shooter/Delivery", 80, false);
        public static final LoggedTunableNumber kDump = new LoggedTunableNumber("Shooter/Dump", 50, false);

        public static final LoggedTunableNumber kDeliveryYThreshold = new LoggedTunableNumber("Shooter/MaxDelieveryOffset", 5, false);

        private static final InterpolatingDoubleTreeMap kShootMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kShoot0 = new LoggedTunableNumber("Shooter/Shoot/0", 44, true);
        private static final LoggedTunableNumber kShoot2 = new LoggedTunableNumber("Shooter/Shoot/2", 50, true);
        private static final LoggedTunableNumber kShoot225 = new LoggedTunableNumber("Shooter/Shoot/2.25", 51, true);
        private static final LoggedTunableNumber kShoot25 = new LoggedTunableNumber("Shooter/Shoot/2.5", 53, true);
        private static final LoggedTunableNumber kShoot275 = new LoggedTunableNumber("Shooter/Shoot/2.75", 54, true);
        private static final LoggedTunableNumber kShoot3 = new LoggedTunableNumber("Shooter/Shoot/3", 55, true);
        private static final LoggedTunableNumber kShoot325 = new LoggedTunableNumber("Shooter/Shoot/3.25", 57, true);
        private static final LoggedTunableNumber kShoot35 = new LoggedTunableNumber("Shooter/Shoot/3.5", 60, true);
        private static final LoggedTunableNumber kShoot375 = new LoggedTunableNumber("Shooter/Shoot/3.75", 61, true);
        private static final LoggedTunableNumber kShoot4 = new LoggedTunableNumber("Shooter/Shoot/4", 64, true);
        private static final LoggedTunableNumber kShoot425 = new LoggedTunableNumber("Shooter/Shoot/4.25", 65, true);
        private static final LoggedTunableNumber kShoot45 = new LoggedTunableNumber("Shooter/Shoot/4.5", 69, true);
        private static final LoggedTunableNumber kShoot475 = new LoggedTunableNumber("Shooter/Shoot/4.75", 71, true);
        private static final LoggedTunableNumber kShoot5 = new LoggedTunableNumber("Shooter/Shoot/5", 76, true);
        private static final LoggedTunableNumber kShoot6 = new LoggedTunableNumber("Shooter/Shoot/6", 86, true);

        public static double getShootSpeed(double dist) {
            kShootMap.put(0.0, kShoot0.get());
            kShootMap.put(2.0, kShoot2.get());
            kShootMap.put(2.25, kShoot225.get());
            kShootMap.put(2.5, kShoot25.get());
            kShootMap.put(2.75, kShoot275.get());
            kShootMap.put(3.0, kShoot3.get());
            kShootMap.put(3.25, kShoot325.get());
            kShootMap.put(3.5, kShoot35.get());
            kShootMap.put(3.75, kShoot375.get());
            kShootMap.put(4.0, kShoot4.get());
            kShootMap.put(4.25, kShoot425.get());
            kShootMap.put(4.5, kShoot45.get());
            kShootMap.put(4.75, kShoot475.get());
            kShootMap.put(5.0, kShoot5.get());
            kShootMap.put(6.0, kShoot6.get());

            return kShootMap.get(dist);
        }

        private static final InterpolatingDoubleTreeMap kAirTimeMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kAirTime0 = new LoggedTunableNumber( "Shooter/Air Time/0",   0, true);
        private static final LoggedTunableNumber kAirTime2 = new LoggedTunableNumber("Shooter/Air Time/2", 0.92, true);
//        private static final LoggedTunableNumber kAirTime225 = new LoggedTunableNumber( "Shooter/Air Time/2.25",   0.81, true);
        private static final LoggedTunableNumber kAirTime25 = new LoggedTunableNumber("Shooter/Air Time/2.5", 1.1, true);
//        private static final LoggedTunableNumber kAirTime275 = new LoggedTunableNumber( "Shooter/Air Time/2.75",   0.96, true);
        private static final LoggedTunableNumber kAirTime3 = new LoggedTunableNumber("Shooter/Air Time/3", 1.1, true);
//        private static final LoggedTunableNumber kAirTime325 = new LoggedTunableNumber( "Shooter/Air Time/3.25",   0.93, true);
        private static final LoggedTunableNumber kAirTime35 = new LoggedTunableNumber("Shooter/Air Time/3.5", 1.17, true);
//        private static final LoggedTunableNumber kAirTime375 = new LoggedTunableNumber( "Shooter/Air Time/3.75",   1, true);
        private static final LoggedTunableNumber kAirTime4 = new LoggedTunableNumber( "Shooter/Air Time/4",   1.24, true);
//        private static final LoggedTunableNumber kAirTime425 = new LoggedTunableNumber( "Shooter/Air Time/4.25",   1.11, true);
        private static final LoggedTunableNumber kAirTime45 = new LoggedTunableNumber( "Shooter/Air Time/4.5",   1.31, true);
//        private static final LoggedTunableNumber kAirTime475 = new LoggedTunableNumber( "Shooter/Air Time/4.75",   1.18, true);
        private static final LoggedTunableNumber kAirTime5 = new LoggedTunableNumber( "Shooter/Air Time/5",   1.3, true);

        public static double getAirTime(double dist) {
            kAirTimeMap.put(0.0,  kAirTime0.get());
            kAirTimeMap.put(2.0,  kAirTime2.get());
//            kAirTimeMap.put(2.25, kAirTime225.get());
            kAirTimeMap.put(2.5,  kAirTime25.get());
//            kAirTimeMap.put(2.75, kAirTime275.get());
            kAirTimeMap.put(3.0,  kAirTime3.get());
//            kAirTimeMap.put(3.25, kAirTime325.get());
            kAirTimeMap.put(3.5,  kAirTime35.get());
//            kAirTimeMap.put(3.75, kAirTime375.get());
            kAirTimeMap.put(4.0,  kAirTime4.get());
//            kAirTimeMap.put(4.25, kAirTime425.get());
            kAirTimeMap.put(4.5,  kAirTime45.get());
//            kAirTimeMap.put(4.75, kAirTime475.get());
            kAirTimeMap.put(5.0,  kAirTime5.get());

            return kAirTimeMap.get(dist);
        }

        private static final InterpolatingDoubleTreeMap kDeliveryMap = new InterpolatingDoubleTreeMap();
        private static final LoggedTunableNumber kDelivery0 = new LoggedTunableNumber("Shooter/Delivery/0", 22, false);
        private static final LoggedTunableNumber kDelivery05 = new LoggedTunableNumber("Shooter/Delivery/0.5", 25, false);
        private static final LoggedTunableNumber kDelivery1 = new LoggedTunableNumber("Shooter/Delivery/1", 28, false);
        private static final LoggedTunableNumber kDelivery15 = new LoggedTunableNumber("Shooter/Delivery/1.5", 31, false);
        private static final LoggedTunableNumber kDelivery2 = new LoggedTunableNumber("Shooter/Delivery/2", 34, false);
        private static final LoggedTunableNumber kDelivery25 = new LoggedTunableNumber("Shooter/Delivery/2.5", 37, false);
        private static final LoggedTunableNumber kDelivery3 = new LoggedTunableNumber("Shooter/Delivery/3", 40, false);
        private static final LoggedTunableNumber kDelivery35 = new LoggedTunableNumber("Shooter/Delivery/3.5", 43, false);
        private static final LoggedTunableNumber kDelivery4 = new LoggedTunableNumber("Shooter/Delivery/4", 46, false);
        private static final LoggedTunableNumber kDelivery45 = new LoggedTunableNumber("Shooter/Delivery/4.5", 49, false);
        private static final LoggedTunableNumber kDelivery5 = new LoggedTunableNumber("Shooter/Delivery/5", 52, false);
        private static final LoggedTunableNumber kDelivery55 = new LoggedTunableNumber("Shooter/Delivery/5.5", 55, false);
        private static final LoggedTunableNumber kDelivery6 = new LoggedTunableNumber("Shooter/Delivery/6", 58, false);

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
            kDeliveryMap.put(4.5, kDelivery45.get());
            kDeliveryMap.put(5.0, kDelivery5.get());
            kDeliveryMap.put(5.5, kDelivery55.get());
            kDeliveryMap.put(6.0, kDelivery6.get());

            return kDeliveryMap.get(dist);
        }
    }

    public static class Accelerator {
        public static final LoggedTunableNumber kAccelerate = new LoggedTunableNumber("Accelerator/Accelerate", 80, false);
    }

    public static class Swerve {
        public static final LoggedTunableNumber kPositionThreshold = new LoggedTunableNumber("Swerve/Pos Thresh", 0.05, false);
//        public static final LoggedTunableNumber kAngleThreshold = new LoggedTunableNumber("Swerve/Angle Thresh", 1, false);
        public static final LoggedTunableNumber kAngleBaseThreshold = new LoggedTunableNumber("Swerve/Angle Base Threshold", 7.5, false);
        public static final LoggedTunableNumber kAngleCoefficient = new LoggedTunableNumber("Swerve/Angle Coefficient", -1.2, false);
        public static final LoggedTunableNumber kSpeedDifferenceThreshold = new LoggedTunableNumber("Swerve/Speed Diff Thresh", 0.3, false);
        public static final LoggedTunableNumber kTargetMinThreshold = new LoggedTunableNumber("Swerve/Target Min Thresh", 1.25, false);

        public static final LoggedTunableNumber kHubMinDist = new LoggedTunableNumber("Swerve/Hub Min Dist", 2, false);
        public static final LoggedTunableNumber kHubMaxDist = new LoggedTunableNumber("Swerve/Hub Max Dist", 3, false);

        private static final LoggedTunableNumber kDeliveryTargetX = new LoggedTunableNumber("Swerve/Delivery Target X", 3, false);
        private static final LoggedTunableNumber kDeliveryTargetY = new LoggedTunableNumber("Swerve/Delivery Target Y", 3, false);

        public static Pose2d getDeliveryTarget() {
            return new Pose2d(kDeliveryTargetX.get(), kDeliveryTargetY.get(), Rotation2d.kZero);
        }
    }
}
