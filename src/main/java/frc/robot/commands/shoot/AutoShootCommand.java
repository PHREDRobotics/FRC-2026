package frc.robot.commands.shoot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoShootCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private FuelSubsystem m_fuelSubsystem;
  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;

  private double m_x;
  private double m_y;

  public AutoShootCommand(ShooterSubsystem shooterSubsystem, FuelSubsystem fuelSubsystem, SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem, DoubleSupplier x, DoubleSupplier y) {
    m_shooterSubsystem = shooterSubsystem;
    m_fuelSubsystem = fuelSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;

    m_x = x.getAsDouble();
    m_y = y.getAsDouble();

    addRequirements(shooterSubsystem, swerveSubsystem, visionSubsystem);
  }

  private boolean canShoot() {
    return m_swerveSubsystem.isAlignedWithHub() && m_shooterSubsystem.isAtSpeed();
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.shoot(m_shooterSubsystem.getShootPower(m_swerveSubsystem.getHubDistance()));
  }

  @Override
  public void execute() {
    if (canShoot()) {
      m_fuelSubsystem.feed();
    }

    m_swerveSubsystem.alignToAndDrive(m_x, m_y, new Rotation2d(m_swerveSubsystem.getPointAngleRadians(Constants.VisionConstants.kHubPos)), false);

    if (m_visionSubsystem.hasValidTarget()) {
      m_swerveSubsystem.addVisionMeasurement(m_visionSubsystem.getEstimatedGlobalPose().get().estimatedPose.toPose2d(), Timer.getFPGATimestamp());
    }
  }
}