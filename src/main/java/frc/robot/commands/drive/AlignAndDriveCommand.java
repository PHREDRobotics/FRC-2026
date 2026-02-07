package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AlignAndDriveCommand extends Command {
  SwerveSubsystem m_swerveSubsystem;

  double m_x;
  double m_y;

  Translation2d m_position;

  public AlignAndDriveCommand(SwerveSubsystem swerveSubsystem, Translation2d position, DoubleSupplier x, DoubleSupplier y) {
    m_swerveSubsystem = swerveSubsystem;

    m_x = x.getAsDouble();
    m_y = y.getAsDouble();

    m_position = position;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void execute() {
    m_swerveSubsystem.alignToAndDrive(m_x, m_y, new Rotation2d(m_swerveSubsystem.getPointAngleRadians(m_position)), false);
  }
}
