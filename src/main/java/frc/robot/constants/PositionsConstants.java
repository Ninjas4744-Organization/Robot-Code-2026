package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.NinjasLib.LoggedTunableNumber;

public class PositionsConstants {
    public static class Arm {
        private static final double DOWN_ANGLE = -30;

        public static final LoggedTunableNumber CLOSE =
            new LoggedTunableNumber("Arm/Close", -90, false);

        public static final LoggedTunableNumber CORAL_OUTTAKE =
            new LoggedTunableNumber("Arm/L", -315, false);

        public static final LoggedTunableNumber CORAL_OUTTAKE_INVERSE =
            new LoggedTunableNumber("Arm/LInverse", -225, false);

        public static final LoggedTunableNumber INTAKE_ALGAE_FLOOR =
            new LoggedTunableNumber("Arm/IntakeAlgaeFloor", -15, false);

        public static final LoggedTunableNumber INTAKE_ALGAE_REEF =
            new LoggedTunableNumber("Arm/IntakeAlgaeReef", 0, false);

        public static final LoggedTunableNumber ALGAE_IN_OUTTAKE =
            new LoggedTunableNumber("Arm/AlgaeInOuttake", 90, false);

        public static final LoggedTunableNumber NET =
            new LoggedTunableNumber("Arm/Net", 110, false);

        public static final LoggedTunableNumber NET_INVERSE =
            new LoggedTunableNumber("Arm/NetInverse", 70, false);

        public static final LoggedTunableNumber INTAKE_CORAL =
            new LoggedTunableNumber("Arm/IntakeCoral", -90, false);

        public static final LoggedTunableNumber CORAL_READY =
            new LoggedTunableNumber("Arm/CoralReady", -270, false);

        public static Rotation2d getLPosition(int L, boolean inverse, boolean down) {
            double angle;

            switch (L) {
                case 1:
                    angle = CLOSE.get();
                    break;

                case 2, 4, 3:
                    angle = inverse ? CORAL_OUTTAKE_INVERSE.get() : CORAL_OUTTAKE.get();
                    break;

                default:
                    angle = CLOSE.get();
                    break;
            }

            if (down && L != 1) {
                angle += inverse ? -DOWN_ANGLE : DOWN_ANGLE;
            }

            return Rotation2d.fromDegrees(angle);
        }
    }
    
    
    
    public static class Elevator {
        private static final double DOWN_OFFSET = 1.0;

        public static final LoggedTunableNumber CLOSE =
            new LoggedTunableNumber("Elevator/Close", 6.5, false);

        public static final LoggedTunableNumber CORAL_OUTTAKE_L2 =
            new LoggedTunableNumber("Elevator/CoralOuttake/L2", 2.25, false);

        public static final LoggedTunableNumber CORAL_OUTTAKE_L3 =
            new LoggedTunableNumber("Elevator/CoralOuttake/L3", 5.5, false);

        public static final LoggedTunableNumber CORAL_OUTTAKE_L4 =
            new LoggedTunableNumber("Elevator/CoralOuttake/L4", 10.7, false);

        public static final LoggedTunableNumber ALGAE_REEF_HIGH =
            new LoggedTunableNumber("Elevator/AlgaeReefHigh", 7.5, false);

        public static final LoggedTunableNumber ALGAE_REEF_LOW =
            new LoggedTunableNumber("Elevator/AlgaeReefLow", 4.3, false);

        public static final LoggedTunableNumber NET =
            new LoggedTunableNumber("Elevator/Net", 10.7, false);

        public static final LoggedTunableNumber ALGAE_FLOOR =
            new LoggedTunableNumber("Elevator/AlgaeFloor", 0.3, false);

        public static final LoggedTunableNumber INTAKE =
            new LoggedTunableNumber("Elevator/Intake", 6.0, false);

        public static final LoggedTunableNumber CORAL_READY =
            new LoggedTunableNumber("Elevator/CoralReady", 1.6, false);

        public static double getLPosition(int L, boolean down) {
            double height;

            switch (L) {
                case 1:
                    height = CLOSE.get();
                    break;

                case 2:
                    height = CORAL_OUTTAKE_L2.get();
                    break;

                case 3:
                    height = CORAL_OUTTAKE_L3.get();
                    break;

                case 4:
                    height = CORAL_OUTTAKE_L4.get();
                    break;

                default:
                    height = CLOSE.get();
                    break;
            }

            if (down && L != 1) {
                height -= DOWN_OFFSET;
            }

            return height;
        }
    }
    
    

    public static class Outtake {
        public static final LoggedTunableNumber CURRENT_THRESHOLD =
            new LoggedTunableNumber("Outtake/CurrentThreshold", 50, false);

        public static final LoggedTunableNumber WAIT_TIME_ALGAE_OUTTAKE =
            new LoggedTunableNumber("Outtake/WaitTimeAlgaeOuttake", 0.2, false);

        public static final LoggedTunableNumber INTAKE =
            new LoggedTunableNumber("Outtake/IntakeSpeed", -0.8, false);

        public static final LoggedTunableNumber OUTTAKE =
            new LoggedTunableNumber("Outtake/OuttakeSpeed", 0.5, false);

        public static final LoggedTunableNumber INTAKE_ALGAE =
            new LoggedTunableNumber("Outtake/IntakeAlgaeSpeed", -1.0, false);

        public static final LoggedTunableNumber OUTTAKE_ALGAE =
            new LoggedTunableNumber("Outtake/OuttakeAlgaeSpeed", 1.0, false);
    }
    
    

    public static class Intake {
        public static final LoggedTunableNumber INTAKE =
            new LoggedTunableNumber("Intake/IntakeSpeed", -70, false);

        public static final LoggedTunableNumber OUTTAKE =
            new LoggedTunableNumber("Intake/OuttakeSpeed", 60, false);

        public static final LoggedTunableNumber OUTTAKE_L1 =
            new LoggedTunableNumber("Intake/OuttakeL1Speed", 26, false);
    }
    
    

    public static class IntakeAngle {
        public static final LoggedTunableNumber INTAKE =
            new LoggedTunableNumber("IntakeAngle/Intake", -23 - 35 - 14 + 5 - 0.5, false);

        public static final LoggedTunableNumber L1 =
            new LoggedTunableNumber("IntakeAngle/L1", 34, false);

        public static final LoggedTunableNumber ARM =
            new LoggedTunableNumber("IntakeAngle/Arm", 65 - 0.5, false);

        public static final LoggedTunableNumber CLOSE =
            new LoggedTunableNumber("IntakeAngle/Close", 65 - 0.5, false);

        public static final LoggedTunableNumber ALGAE =
            new LoggedTunableNumber("IntakeAngle/Algae", 0, false);
    }
    
    

    public static class IntakeAligner {
        public static final LoggedTunableNumber ALIGN =
            new LoggedTunableNumber("IntakeAligner/AlignSpeed", 100, false);
    }
    
    
    
    public static class AutoDrive {
        public static final LoggedTunableNumber DIST_FROM_REEF =
            new LoggedTunableNumber("AutoDrive/DistFromReef", 0.6, false);

        public static final LoggedTunableNumber RIGHT_OFFSET =
            new LoggedTunableNumber("AutoDrive/RightOffset", -0.3, false);

        public static final LoggedTunableNumber LEFT_OFFSET =
            new LoggedTunableNumber("AutoDrive/LeftOffset", 0.3, false);

        public static final LoggedTunableNumber DIST_FROM_REEF_INVERSE =
            new LoggedTunableNumber("AutoDrive/DistFromReefInverse", 0.6, false);

        public static final LoggedTunableNumber RIGHT_OFFSET_INVERSE =
            new LoggedTunableNumber("AutoDrive/RightOffsetInverse", -0.3, false);

        public static final LoggedTunableNumber LEFT_OFFSET_INVERSE =
            new LoggedTunableNumber("AutoDrive/LeftOffsetInverse", 0.3, false);

        public static final LoggedTunableNumber POSITION_THRESHOLD =
            new LoggedTunableNumber("AutoDrive/PositionThreshold", 0.03, false);

        public static final LoggedTunableNumber ROTATION_THRESHOLD_DEG =
            new LoggedTunableNumber("AutoDrive/RotationThresholdDeg", 4.0, false);
    }
}
