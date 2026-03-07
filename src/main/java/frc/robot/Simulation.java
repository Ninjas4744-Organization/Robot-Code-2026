package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.swerve.Swerve;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

import static edu.wpi.first.units.Units.*;

public class Simulation {
    private static List<GamePieceProjectile> simFlyingBalls = new ArrayList<>();
    private static List<GamePieceOnFieldSimulation> simFieldBalls = new ArrayList<>();
    private static int ballsCollected = 0;
    private static int points = 0;
    private static int ballZigZagNum = 0;

    private static void spawnBalls() {
        for (int x = 7 * 5; x <= 10 * 5; x++) {
            for (int y = 2 * 5; y <= 6 * 5; y++) {
                RebuiltFuelOnField ball = new RebuiltFuelOnField(new Translation2d(x / 5.0, y / 5.0));
                SimulatedArena.getInstance().addGamePiece(ball);
                simFieldBalls.add(ball);
            }
        }
    }

    public static void reset() {
        points = 0;
        ballsCollected = 0;
        spawnBalls();
        Logger.recordOutput("Simulation/Points", points);
        Logger.recordOutput("Simulation/Balls Collected", ballsCollected);
    }

    public static void setup() {
        reset();
        spawnBalls();

        CommandScheduler.getInstance().schedule(Commands.runOnce(() -> {
            if (ballsCollected > 0 && Math.abs(RobotContainer.getShooter().getVelocity()) > 10 && RobotState.get().getRobotState() == States.SHOOT) {
                ballsCollected--;
                Logger.recordOutput("Simulation/Balls Collected", ballsCollected);

                GamePieceProjectile ball = new RebuiltFuelOnFly(
                    RobotState.get().getRobotPose().getTranslation(),
                    new Translation2d(0.2, ballZigZagNum % 2 == 0 ? 0.1 : -0.1),
                    Swerve.getInstance().getSpeeds().getAsFieldRelative(),
                    RobotState.get().getRobotPose().getRotation(),
                    Meters.of(0.481),
                    MetersPerSecond.of(0.062 * Math.abs(RobotContainer.getShooter().getGoal()) + 3.3),
                    Degrees.of(60)
                );
                ballZigZagNum = (ballZigZagNum + 1) % 2;

                simFlyingBalls.add(ball);
                SimulatedArena.getInstance().addGamePieceProjectile(ball);
            }
        }).andThen(Commands.waitSeconds(0.06)).repeatedly().ignoringDisable(true));
    }

    public static void periodic() {
        RobotState.get().resetGyro(RobotState.get().getRotation());

        SimulatedArena.getInstance().simulationPeriodic();

        if (simFieldBalls.size() <= 250)
            spawnBalls();

        var flyingBallsIter = simFlyingBalls.iterator();
        while (flyingBallsIter.hasNext()) {
            GamePieceProjectile ball = flyingBallsIter.next();
            Pose3d ballPose = ball.getPose3d();
            double ballRadius = 0.07;

            if (simBallColliding(new Translation2d(4.63, 4.03), 0.8, 0.8, ballPose.toPose2d().getTranslation(), 0.07) && ball.getPose3d().getZ() > 1.6 && ball.getPose3d().getZ() < 1.85) {
                RebuiltFuelOnField newBall = new RebuiltFuelOnField(new Translation2d(ball.getPose3d().getX() + 1, ball.getPose3d().getY()));
                SimulatedArena.getInstance().addGamePiece(newBall);
                simFieldBalls.add(newBall);
                points++;
                Logger.recordOutput("Simulation/Points", points);

                flyingBallsIter.remove();
            } else if (ballPose.getZ() <= ballRadius) {
                RebuiltFuelOnField newBall = new RebuiltFuelOnField(new Translation2d(ball.getPose3d().getX(), ball.getPose3d().getY()));
                SimulatedArena.getInstance().addGamePiece(newBall);
                simFieldBalls.add(newBall);

                flyingBallsIter.remove();
            }
        }

        if (RobotContainer.getIntake().getVelocity() > 10 && ballsCollected < 40) {
            var fieldBallsIter = simFieldBalls.iterator();
            while (fieldBallsIter.hasNext()) {
                GamePieceOnFieldSimulation ball = fieldBallsIter.next();

                Rotation2d robotRotation = RobotState.get().getRotation();
                Translation2d robotCenter = RobotState.get().getRobotPose().getTranslation();

                double intakeOffsetDist = -0.51855 / 2.0 - 0.25;
                double intakeWidth = 0.59475 + 0.2;
                double intakeHeight = 0.30;
                double ballRadius = 0.07;

                Translation2d intakeCenter = robotCenter.plus(
                    new Translation2d(intakeOffsetDist, 0).rotateBy(robotRotation)
                );

    //            Translation2d[] vertices = new Translation2d[] {
    //                new Translation2d(intakeHeight / 2.0, intakeWidth / 2.0).rotateBy(robotRotation).plus(intakeCenter),
    //                new Translation2d(intakeHeight / 2.0, -intakeWidth / 2.0).rotateBy(robotRotation).plus(intakeCenter),
    //                new Translation2d(-intakeHeight / 2.0, -intakeWidth / 2.0).rotateBy(robotRotation).plus(intakeCenter),
    //                new Translation2d(-intakeHeight / 2.0, intakeWidth / 2.0).rotateBy(robotRotation).plus(intakeCenter)
    //            };
    //            Logger.recordOutput("Simulation/IntakeHitbox", vertices);

                if (simBallColliding(intakeCenter, intakeWidth, intakeHeight, ball.getPose3d().toPose2d().getTranslation(), ballRadius)) {
                    SimulatedArena.getInstance().removePiece(ball);
                    fieldBallsIter.remove();
                    ballsCollected++;
                    Logger.recordOutput("Simulation/Balls Collected", ballsCollected);
                }
            }
        }

        for (int i = 0; i < simFieldBalls.size() - 400; i++) {
            SimulatedArena.getInstance().removePiece(simFieldBalls.get(0));
            simFieldBalls.remove(0);
        }

        Pose3d[] balls = new Pose3d[simFlyingBalls.size() + simFieldBalls.size()];
        for (int i = 0; i < simFlyingBalls.size(); i++) {
            balls[i] = simFlyingBalls.get(i).getPose3d();
        }
        for (int i = 0; i < simFieldBalls.size(); i++) {
            balls[simFlyingBalls.size() + i] = simFieldBalls.get(i).getPose3d();
        }

        Logger.recordOutput("Robot/Shooting/Balls", balls);
    }

    private static boolean simBallColliding(
        Translation2d rectCenter, double rectWidth, double rectHeight,
        Translation2d circleCenter, double circleRadius) {

        // 1. Find the boundaries of the rectangle
        double rectMinX = rectCenter.getX() - (rectWidth / 2.0);
        double rectMaxX = rectCenter.getX() + (rectWidth / 2.0);
        double rectMinY = rectCenter.getY() - (rectHeight / 2.0);
        double rectMaxY = rectCenter.getY() + (rectHeight / 2.0);

        // 2. "Clamp" the circle's center to the rectangle's limits
        // This finds the point on the rectangle closest to the circle center
        double closestX = Math.max(rectMinX, Math.min(circleCenter.getX(), rectMaxX));
        double closestY = Math.max(rectMinY, Math.min(circleCenter.getY(), rectMaxY));

        // 3. Calculate distance from circle center to this closest point
        double distanceX = circleCenter.getX() - closestX;
        double distanceY = circleCenter.getY() - closestY;

        // Use Pythagorean theorem squared to avoid expensive square root calculations
        double distanceSquared = (distanceX * distanceX) + (distanceY * distanceY);

        return distanceSquared < (circleRadius * circleRadius);
    }
}
