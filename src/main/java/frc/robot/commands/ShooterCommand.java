package frc.robot.commands;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.google.errorprone.annotations.Var;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.Constants;

public class ShooterCommand extends Command {
  private ShooterSubsystem m_shooterSubsystem;

  public void ShooterCommand(ShooterSubsystem subsystem) {
    m_shooterSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    m_shooterSubsystem.shootMotorsStart(Constants.ShooterConstants.kInitialShootingSpeed);
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
