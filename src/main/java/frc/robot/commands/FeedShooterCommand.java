package frc.robot.commands;

import frc.robot.subsystems.Fuel.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

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

  @Override
  public void execute() { 
  }

  @Override
  public void end(boolean interrupted) {
    FeederSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
}