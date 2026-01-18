package frc.robot.commands;

import frc.robot.subsystems.Fuel.HopperSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class HopperCommand extends Command {
    private HopperSubsystem HopperSubsystem;

  /**
   * Creates a new HopperExtendCommand.
   * 
   * @param subsystem Hopper subsystem
   */
  public HopperCommand(HopperSubsystem subsystem) {
    HopperSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    HopperSubsystem.startHopper();
  }
  
  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    HopperSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}