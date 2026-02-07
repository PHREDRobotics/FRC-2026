package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class GoToPoseCommand extends Command {

  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private Pose2d pose;

  public GoToPoseCommand(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, Pose2d pose) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.m_visionSubsystem = visionSubsystem;

    this.pose = pose;

    addRequirements(swerveSubsystem);
    addRequirements(visionSubsystem);
  }

  @Override
  public void initialize() {
    m_swerveSubsystem.resetPIDs(new Pose2d());
  }

  @Override
  public void execute() {
    if (m_visionSubsystem.hasValidTarget()) {
      m_swerveSubsystem.driveTo(pose);

      // Send vision measurements
      m_swerveSubsystem.addVisionMeasurement(m_visionSubsystem.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    } else {
      m_swerveSubsystem.driveRelativeTo(new Pose2d(), new Pose2d());
    }

  }
}