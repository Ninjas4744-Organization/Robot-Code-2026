package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.lib.NinjasLib.LoggedTunableNumber;

public class PositionsConstants {
    public static class ClimberAngle {
        public static final LoggedTunableNumber kClose = new LoggedTunableNumber("Climber Angle/Close", 90, false);
        public static final LoggedTunableNumber kOpen = new LoggedTunableNumber("Climber Angle/Open", 0, false);
    }

    public static class Climber {
        public static final LoggedTunableNumber kRightClimb = new LoggedTunableNumber("Climber/Right Climb", 90, false);
        public static final LoggedTunableNumber kLeftClimb = new LoggedTunableNumber("Climber/Left Climb", -90, false);
    }

    public static class Intake {
        public static final LoggedTunableNumber kIntake = new LoggedTunableNumber("Intake/Intake", 100, false);
    }

    public static class IntakeIndexer {
        public static final LoggedTunableNumber kIntake = new LoggedTunableNumber("Intake Indexer/Intake", 100, false);
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
        private static final LoggedTunableNumber kShoot0 = new LoggedTunableNumber("Shooter/Shoot 0", 60, false);
        private static final LoggedTunableNumber kShoot05 = new LoggedTunableNumber("Shooter/Shoot 0.5", 65, false);
        private static final LoggedTunableNumber kShoot1 = new LoggedTunableNumber("Shooter/Shoot 1", 70, false);
        private static final LoggedTunableNumber kShoot15 = new LoggedTunableNumber("Shooter/Shoot 1.5", 75, false);
        private static final LoggedTunableNumber kShoot2 = new LoggedTunableNumber("Shooter/Shoot 2", 80, false);
        private static final LoggedTunableNumber kShoot25 = new LoggedTunableNumber("Shooter/Shoot 2.5", 85, false);
        private static final LoggedTunableNumber kShoot3 = new LoggedTunableNumber("Shooter/Shoot 3", 90, false);
        private static final LoggedTunableNumber kShoot35 = new LoggedTunableNumber("Shooter/Shoot 3.5", 95, false);
        private static final LoggedTunableNumber kShoot4 = new LoggedTunableNumber("Shooter/Shoot 4", 100, false);

        public static double getShootSpeed(double dist) {
            kShootMap.put(0.0, kShoot0.get());
            kShootMap.put(0.5, kShoot05.get());
            kShootMap.put(1.0, kShoot1.get());
            kShootMap.put(1.5, kShoot15.get());
            kShootMap.put(2.0, kShoot2.get());
            kShootMap.put(2.5, kShoot25.get());
            kShootMap.put(3.0, kShoot3.get());
            kShootMap.put(3.5, kShoot35.get());
            kShootMap.put(4.0, kShoot4.get());

            return kShootMap.get(dist);
        }
    }

    public static class ShooterIndexer {
        public static final LoggedTunableNumber kShoot = new LoggedTunableNumber("ShooterIndexer/Shoot", 100, false);
    }
}
