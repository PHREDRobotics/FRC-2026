// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
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
  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;

  private final AutoFactory autoFactory;

  LogitechPro joystick;
  // CommandJoystick buttonBox;

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem();
    visionSubsystem = new VisionSubsystem();

    autoFactory = new AutoFactory(
      swerveSubsystem::getPose,
      swerveSubsystem::resetOdometry,
      swerveSubsystem::followTrajectory, true, swerveSubsystem);

    joystick = new LogitechPro(0);
    // CommandJoystick buttonBox = new CommandJoystick(1);

    configureBindings(); 
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(new DriveCommand(swerveSubsystem,
      joystick::getY,
      joystick::getX, 
      joystick::getZ,
      joystick::getThrottle,
      joystick::getFieldOriented));

   new Trigger(() -> true) // always active, sends vision estimates to swerve
        .onTrue(new InstantCommand(() -> {
          visionSubsystem.getEstimatedRelativePose().ifPresent(pose -> {
            swerveSubsystem.addVisionMeasurement(pose, Timer.getFPGATimestamp());
          });
        })); 

    //joystick.getAlignTag().onTrue(new AlignTagCommand(swerveSubsystem, visionSubsystem, ));
    //joystick.getFollowTag().onTrue(new FollowTagCommand(swerveSubsystem, visionSubsystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
