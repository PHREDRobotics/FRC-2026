package frc.robot.commands;

import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;

public class MoveIntake extends Command {
    
  private IntakeSubsystem IntakeSubsystem;

  /**
   * Creates a new IntakeCommand.
   * 
   * @param subsystem Intake subsystem
   */
  public MoveIntake(IntakeSubsystem subsystem) {
    IntakeSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    IntakeSubsystem.moveIntake();
  }
  
}