// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command autonomous_command;

  private final RobotContainer robot_container;
  private final Vision vision;

  public Robot() {
    robot_container = new RobotContainer();
    vision = Vision.getInstance();
  }

  @Override
  public void robotPeriodic() {
    // for (EstimatedRobotPose estimated_pose : vision.getCameraPoseEstimations()) {
    //   robot_container.drivetrain.addVisionMeasurement(estimated_pose.estimatedPose.toPose2d(),
    //       estimated_pose.timestampSeconds);
    // }

    CommandScheduler.getInstance().run();
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
    autonomous_command = robot_container.getAutonomousCommand();

    if (autonomous_command != null) {
      autonomous_command.schedule();
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
    if (autonomous_command != null) {
      autonomous_command.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
