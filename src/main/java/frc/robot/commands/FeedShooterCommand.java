package frc.robot.commands;

import frc.robot.subsystems.Fuel.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;

public class FeedShooterCommand extends Command {
    private FeederSubsystem FeederSubsystem;

  /**
   * Creates a new FeedHopperCommand.
   * 
   * @param subsystem Feeder subsystem
   */
  public FeedShooterCommand(FeederSubsystem subsystem) {
    FeederSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    FeederSubsystem.feed();
  }
  
}