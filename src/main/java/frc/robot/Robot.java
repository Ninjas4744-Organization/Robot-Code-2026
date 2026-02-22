// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.NinjasLib.swerve.Swerve;
import frc.robot.constants.GeneralConstants;
import frc.robot.constants.SubsystemConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private final RobotContainer robotContainer;
    public static double teleopStartTime;
    private double lastLoopTime;
    private double robotDt = 0.02;

    public Robot() {
        // Record metadata
        Logger.recordMetadata("ProjectName", "Robot-Code-2026");

        // Set up data receivers & replay source
        switch (GeneralConstants.kRobotMode) {
            case WORKSHOP, COMP, SIM, SIM_COMP:
                // A FAT32 formatted USB stick must be connected to one of the roboRIO USB ports.
                // Running on a real robot, log to a USB stick ("/U/logs")
                // Also runs sim and logs to logs/ folder
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY, REPLAY_COMP:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
                break;
        }

        // Start AdvantageKit logger
        Logger.start();

        robotContainer = new RobotContainer();
    }

    public double getRobotDt() {
        return robotDt;
    }

    @Override
    public void robotPeriodic() {
        robotContainer.controllerPeriodic();
        CommandScheduler.getInstance().run();
        robotContainer.periodic();

        robotDt = RobotController.getFPGATime() / 1000000.0 - lastLoopTime;
        Logger.recordOutput("Robot/Delta Time", getRobotDt());
        lastLoopTime = RobotController.getFPGATime() / 1000000.0;
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        robotContainer.reset();
        autonomousCommand = robotContainer.getAutonomousCommand();

        Swerve.getInstance().setMaxSkidAcceleration(Double.MAX_VALUE);
        Swerve.getInstance().setMaxForwardAcceleration(Double.MAX_VALUE);

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        teleopStartTime = RobotController.getFPGATime();
        robotContainer.reset();

        Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
        Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
//        CommandScheduler.getInstance().cancelAll();

        Swerve.getInstance().setMaxSkidAcceleration(SubsystemConstants.kSwerve.limits.maxSkidAcceleration);
        Swerve.getInstance().setMaxForwardAcceleration(SubsystemConstants.kSwerve.limits.maxForwardAcceleration);

        CommandScheduler.getInstance().schedule(Commands.sequence(
            RobotContainer.getSwerve().reset(),
            RobotContainer.getShooter().stopCmd(),
            RobotContainer.getIndexer().stopCmd(),
            RobotContainer.getIndexer2().stopCmd(),
            RobotContainer.getAccelerator().stopCmd()
        ));
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
