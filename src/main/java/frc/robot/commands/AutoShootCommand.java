package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AutoShootCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private SwerveSubsystem m_swerveSubsystem;
  private VisionSubsystem m_visionSubsystem;
  private DoubleSupplier x;
  private DoubleSupplier y;

  public AutoShootCommand(ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem,
      VisionSubsystem visionSubsystem, DoubleSupplier x, DoubleSupplier y) {
    m_shooterSubsystem = shooterSubsystem;
    m_swerveSubsystem = swerveSubsystem;
    m_visionSubsystem = visionSubsystem;
    this.x = x;
    this.y = y;

    addRequirements(shooterSubsystem, swerveSubsystem, visionSubsystem);
  }


  @Override
  public void initialize() {
    new ShooterCommand(m_shooterSubsystem, m_visionSubsystem::getShootPower);
  }

  @Override
    public void execute() {

        SmartDashboard.putNumber("Shoot power", m_visionSubsystem.getShootPower());
    }
}

/*
 * Auto shoot pseudo-code
 * 
 * power = k * distance^n
 * 
 * n = 1.5?
 * 
 * k = max_power / max_distance
 * 
 * k = 1/5?
 * 
 * shoot(power)
 */
