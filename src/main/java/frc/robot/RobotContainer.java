// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  private final SwerveSubsystem m_swerveSubsystem;
  private final VisionSubsystem m_visionSubsystem;
  private final ShooterSubsystem m_shooterSubsystem;

  private final AutoFactory autoFactory;

  LogitechPro joystick;

  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();
    m_visionSubsystem = new VisionSubsystem();
    m_shooterSubsystem = new ShooterSubsystem();

    autoFactory = new AutoFactory(
      m_swerveSubsystem::getPose,
      m_swerveSubsystem::resetOdometry,
      m_swerveSubsystem::followTrajectory, true, m_swerveSubsystem);

    joystick = new LogitechPro(0);

  
    configureBindings(); 
  }

  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(new DriveCommand(m_swerveSubsystem,
      joystick::getY,
      joystick::getX, 
      joystick::getZ,
      joystick::getThrottle,
      joystick.button(1)));

   new Trigger(() -> true) // always active, sends vision estimates to swerve
        .onTrue(new InstantCommand(() -> {
          m_visionSubsystem.getEstimatedRelativePose().ifPresent(pose -> {
            m_swerveSubsystem.addVisionMeasurement(pose, Timer.getFPGATimestamp());
          });
        })); 

    Trigger shooterButton = new Trigger(joystick.button(1));
    shooterButton.onTrue(new ShooterCommand(m_shooterSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
