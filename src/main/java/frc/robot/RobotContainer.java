// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignTagCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FollowTagCommand;
import frc.robot.controls.LogitechPro;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class RobotContainer {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;

  LogitechPro m_joystick;
  // CommandJoystick m_buttonBox;

  public RobotContainer() {
    m_swerveSubsystem = new SwerveSubsystem();
    m_visionSubsystem = new VisionSubsystem();

    LogitechPro m_joystick = new LogitechPro(0);
    // CommandJoystick m_buttonBox = new CommandJoystick(1);

    configureBindings(); 
  }

  private void configureBindings() {
    m_swerveSubsystem.setDefaultCommand(new DriveCommand(m_swerveSubsystem,
      () -> m_joystick.getY(),
      () -> m_joystick.getX(), 
      () -> m_joystick.getZ(),
      () -> m_joystick.getThrottle(),
      () -> m_joystick.getFieldOriented()));

   new Trigger(() -> true) // always active, sends vision estimates to swerve
        .onTrue(new InstantCommand(() -> {
          m_visionSubsystem.getEstimatedRelativePose().ifPresent(pose -> {
            m_swerveSubsystem.addVisionMeasurement(pose, Timer.getFPGATimestamp());
          });
        })); 

    m_joystick.getAlignTag().onTrue(new AlignTagCommand(m_swerveSubsystem, m_visionSubsystem));
    m_joystick.getFollowTag().onTrue(new FollowTagCommand(m_swerveSubsystem, m_visionSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
