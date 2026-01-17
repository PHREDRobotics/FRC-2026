package frc.robot.commands;

import frc.robot.subsystems.Fuel.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class ExtendHopper extends Command {
    
  private HopperSubsystem hopperSubsystem;

  /**
   * Creates a new ExtendHopperCommand.
   * 
   * @param subsystem Hopper subsystem
   */
  
  public ExtendHopper(HopperSubsystem subsystem) {
    hopperSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    hopperSubsystem.extendHopper();
  }
  
}