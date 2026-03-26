package frc.robot.constants;

import frc.robot.Robot;

public class GeneralConstants {
    public enum RobotMode {
        /** Running on a real robot */
        WORKSHOP,

        /** Running on a real robot in competition */
        COMP,

        /** Running on a simulator */
        SIM,

        /** Running on a simulator in competition */
        SIM_COMP,

        /** Replaying from a log file */
        REPLAY,

        /** Replaying from a competition log file */
        REPLAY_COMP;

        public boolean isReal() {
            return this == WORKSHOP || this == COMP;
        }

        public boolean isSim() {
            return this == SIM || this == SIM_COMP;
        }

        public boolean isReplay() {
            return this == REPLAY || this == REPLAY_COMP;
        }

        public boolean isComp() {
            return this == COMP || this == REPLAY_COMP || this == SIM_COMP;
        }
    }

    private static final RobotMode kSimMode = RobotMode.SIM;
    private static final RobotMode kRealMode = RobotMode.COMP;
    public static final RobotMode kRobotMode = Robot.isReal() ? kRealMode : kSimMode;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final boolean kEnableAutoTiming = false;
    public static final double kAutoTimingSeconds = 3;
    public static final double kAutoTimingStopDeliverySeconds = 6;

    private static final int kFufuCount = 3;
    private static final int kShootCount = 2;
    public static double[] kTasksTimings = new double[] { 129, 118, 105, 100, 90, 79, 68, 55, 0 };
    public static final String[] KTasksNames = new String[] { "SHIFT_1_FUFU_1", "SHIFT_1_FUFU_2", "SHIFT_1_FILL", "SHIFT_2_SHOOT_1", "SHIFT_2_SHOOT_2", "SHIFT_3_FUFU_1", "SHIFT_3_FUFU_2", "SHIFT_3_FILL", "END_SHOOT" };
//    static {
//        double current = 140;
//        for (int i = 0; i < kFufuCount; i++) {
//            current = 35 / kFufuCount;
//            kTasksTimings[i] = 140 -
//        }
//    }

    public static class Swerve {
        public static double kDriverSpeedFactor = 1;
        public static double kDriverRotationSpeedFactor = 1;
        public static double kDriverPowFactor = 0.75;
        public static double kDriverRotationPowFactor = 0.75;
        public static double kJoystickDeadband = 0.04;
        public static boolean kDriverFieldRelative = true;
    }

    public static class Vision {
        public static final double kMaxDistanceFilter = 5;
        public static final double kMinDistanceFilter = 0.6;
        public static final double kMaxSpeedFilter = 3;
        public static final double kMaxAngularSpeedFilter = 3;
        public static final double kMaxAmbiguityFilter = 0.2;
        public static final double kOdometryDriftPerMeter = 0.02316;
    }
}
