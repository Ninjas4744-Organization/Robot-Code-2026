package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;

public class PositionsConstants {
    public static class Arm {
        private static final double DownAngle = -30;
        public static final Rotation2d[] LPositions = { Rotation2d.fromDegrees(Positions.Close.get()), Rotation2d.fromDegrees(Positions.L2.get()), Rotation2d.fromDegrees(Positions.L3.get()), Rotation2d.fromDegrees(Positions.L4.get()) };
        public static final Rotation2d[] LPositionsDown = { Rotation2d.fromDegrees(Positions.Close.get()), Rotation2d.fromDegrees(Positions.L2.get() + DownAngle), Rotation2d.fromDegrees(Positions.L3.get() + DownAngle), Rotation2d.fromDegrees(Positions.L4.get() + DownAngle) };
        public static final Rotation2d[] LPositionsInverse = { Rotation2d.fromDegrees(Positions.Close.get()), Rotation2d.fromDegrees(Positions.L2Inverse.get()), Rotation2d.fromDegrees(Positions.L3Inverse.get()), Rotation2d.fromDegrees(Positions.L4Inverse.get()) };
        public static final Rotation2d[] LPositionsDownInverse = { Rotation2d.fromDegrees(Positions.Close.get()), Rotation2d.fromDegrees(Positions.L2Inverse.get() - DownAngle), Rotation2d.fromDegrees(Positions.L3Inverse.get() - DownAngle), Rotation2d.fromDegrees(Positions.L4Inverse.get() - DownAngle) };
        public enum Positions {
            Close(-90),
            L2(-315),
            L3(-315),
            L4(-315),
            L2Inverse(-225),
            L3Inverse(-225),
            L4Inverse(-225),
            IntakeAlgaeFloor(-15),
            IntakeAlgaeReef(0),
            AlgaeInOuttake(90),
            Net(110),
            NetInverse(70),
            Processor(0),
            IntakeCoral(-90),
            CoralReady(-270);

            final double angle;

            Positions(double angle) {
                this.angle = angle;
            }

            public double get() {
                return angle;
            }
        }
    }
    
    
    
    public static class Elevator {
        public static final double[] LPositions = { Positions.Close.get(), Positions.L2.get(), Positions.L3.get(), Positions.L4.get() };
        public static final double[] LPositionsDown = { Positions.Close.get(), Positions.L2.get() - 1, Positions.L3.get() - 1, Positions.L4.get() - 1 };
        public enum Positions {
            Close(6.5),
            L2(2.25),
            L3(5.5),
            L4(10.7),
            AlgaeReefHigh(7.5),
            AlgaeReefLow(4.3),
            Net(10.7),
            AlgaeFloor(0.3),
            Intake(6),
            CoralReady(1.6);

            final double height;

            Positions(double height) {
                this.height = height;
            }

            public double get() {
                return height;
            }
        }
    }
    
    

    public static class Outtake {
        public static final double kCurrentThreshold = 50;
        public static final double kWaitTimeForAlgaeOuttake = 0.2;

        public enum Speeds {
            Intake(-0.8),
            Outtake(0.5),
            IntakeAlgae(-1),
            OuttakeAlgae(1);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }
    
    

    public static class Intake {
        public enum Speeds {
            Intake(-70),
            Outtake(60),
            OuttakeL1(26);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }
    
    

    public static class IntakeAngle {
        public enum Positions {
            Intake(-23 - 35 - 14 + 5 - 0.5),
            L1(34),
            Arm(65 - 0.5),
            Close(65 - 0.5),
            Algae(0);

            private final double degrees;

            Positions(double degrees) {
                this.degrees = degrees;
            }

            public double get() {
                return degrees;
            }
        }
    }
    
    

    public static class IntakeAligner {
        public enum Speeds {
            Align(100);

            final double speed;

            Speeds(double speed) {
                this.speed = speed;
            }

            public double get() {
                return speed;
            }
        }
    }
    
    
    
    public static class AutoDrive {
        public static final double kDistFromReef = 0.6;
        public static final double kRightOffset = -0.3;
        public static final double kLeftOffset = 0.3;

        public static final double kDistFromReefInverse = 0.6;
        public static final double kRightOffsetInverse = -0.3;
        public static final double kLeftOffsetInverse = 0.3;

        public static final double kPositionThreshold = 0.03;
        public static final Rotation2d kRotationThreshold = Rotation2d.fromDegrees(4);
    }
}
