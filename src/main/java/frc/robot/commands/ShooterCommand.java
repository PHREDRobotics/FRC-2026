package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants;

public class ShooterCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;
  private DoubleSupplier speed;

  public ShooterCommand(ShooterSubsystem subsystem, DoubleSupplier speed) {
    m_shooterSubsystem = subsystem;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.shooterMotorsSet(Constants.ShooterConstants.kInitialShootingSpeed);
  }

  @Override
  public void execute() {
    m_shooterSubsystem.shooterMotorsSet(speed.getAsDouble());
    SmartDashboard.putNumber("Target Speed", Constants.ShooterConstants.kInitialShootingSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
