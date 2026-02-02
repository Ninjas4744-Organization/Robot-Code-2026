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
    private static final RobotMode kRealMode = RobotMode.WORKSHOP;
    public static final RobotMode kRobotMode = Robot.isReal() ? kRealMode : kSimMode;

    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static class Swerve {
        public static double kDriverSpeedFactor = 1;
        public static double kDriverRotationSpeedFactor = 1;
        public static double kJoystickDeadband = 0.04;
        public static boolean kDriverFieldRelative = true;
    }

    public static class Vision {
        public static final double kMaxDistanceFilter = 5;
        public static final double kMinDistanceFilter = 0.6;
        public static final double kMaxSpeedFilter = 2;
        public static final double kMaxAngularSpeedFilter = 2;
        public static final double kMaxAmbiguityFilter = 0.2;
        public static final double kOdometryDriftPerMeter = 0.0177;
    }
}
