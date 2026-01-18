package frc.robot.commands;

import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeDown extends Command {
    
  private IntakeSubsystem IntakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   * 
   * @param subsystem Intake subsystem
   */
  public IntakeDown(IntakeSubsystem subsystem) {
    IntakeSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    IntakeSubsystem.moveIntakeDown();
  }
  
  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.stopIntakeArmMovement();
  }

  @Override
  public boolean isFinished() {
    return IntakeSubsystem.isIntakeDown();
  }
}