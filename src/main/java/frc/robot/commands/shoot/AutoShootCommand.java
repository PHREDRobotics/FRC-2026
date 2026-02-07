package frc.robot.commands.shoot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AutoShootCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private SwerveSubsystem m_swerveSubsystem;

  private double m_x;
  private double m_y;

  public AutoShootCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem, DoubleSupplier x, DoubleSupplier y) {
    m_shooterSubsystem = shooterSubsystem;
    m_swerveSubsystem = swerveSubsystem;

    m_x = x.getAsDouble();
    m_y = y.getAsDouble();

    addRequirements(shooterSubsystem, swerveSubsystem);
  }

  private boolean canShoot() {
    return m_swerveSubsystem.isAligned() && m_shooterSubsystem.isAtSpeed();
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.shoot(m_swerveSubsystem.getShootPower());
  }

  @Override
  public void execute() {
    if (canShoot()) {
      m_shooterSubsystem.feed();
    }

    m_swerveSubsystem.alignToAndDrive(m_x, m_y, new Rotation2d(m_swerveSubsystem.getPointAngleRadians(Constants.VisionConstants.kHubPos)), false);
  }
}