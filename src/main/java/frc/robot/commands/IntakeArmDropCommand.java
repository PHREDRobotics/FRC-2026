package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;

public class IntakeArmDropCommand extends Command {
   private IntakeArmSubsystem m_armSubsystem;

  public void IntakeArmDropCommand(IntakeArmSubsystem subsystem) {
    m_armSubsystem = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_armSubsystem.intakeArmExtend();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
