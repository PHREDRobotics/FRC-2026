package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignTagCommand extends Command {
  SwerveSubsystem m_swerveSubsystem;
  VisionSubsystem m_visionSubsystem;
  double x;
  double y;

  public AlignTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, DoubleSupplier x, DoubleSupplier y) {
    this.m_swerveSubsystem = swerve;
    this.m_visionSubsystem = vision;
    this.x = x.getAsDouble();
    this.y = y.getAsDouble();
  }

  @Override
  public void execute() {
    Pose2d currentPose = this.m_visionSubsystem.getEstimatedRelativePose().get();
    this.m_swerveSubsystem.driveRelativeTo(currentPose,
        new Pose2d(x, y, new Rotation2d(Math.PI)));
  }
}