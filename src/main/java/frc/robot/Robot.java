// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.GoUpAndFall;
// import frc.robot.subsystems.choreo.Drive;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final AutoFactory autoFactory;
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  public Robot() {
    m_robotContainer = new RobotContainer();
autoFactory = new AutoFactory(
            swerveSubsystem::getPose, // A function that returns the current robot pose
            swerveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            swerveSubsystem // The drive subsystem
        );
          }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
