package frc.robot.commands;

import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
    
  private IntakeSubsystem IntakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   * 
   * @param subsystem Intake subsystem
   */
  public IntakeCommand(IntakeSubsystem subsystem) {
    IntakeSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    IntakeSubsystem.startIntake();
  }

  @Override
  public void execute() {
  }
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.stopIntake();
  }
  @Override
  public boolean isFinished() {
    return false;
  }
  
}