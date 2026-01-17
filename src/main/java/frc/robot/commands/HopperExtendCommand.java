package frc.robot.commands;

import frc.robot.subsystems.Fuel.HopperSubsystem;
import frc.robot.subsystems.Fuel.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FuelConstants;

public class HopperExtendCommand extends Command {
    private HopperSubsystem HopperSubsystem;

  /**
   * Creates a new HopperExtendCommand.
   * 
   * @param subsystem Hopper subsystem
   */
  public HopperExtendCommand(HopperSubsystem subsystem) {
    HopperSubsystem = subsystem;
  }

  @Override
  public void initialize() {
    HopperSubsystem.extendHopper();
  }
  
}