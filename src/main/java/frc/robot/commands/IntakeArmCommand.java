package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakeArm.IntakeArmSubsystem;
public class IntakeArmCommand extends Command {

    private IntakeArmSubsystem m_IntakeArmSubsystem;
    
    public IntakeArmCommand(IntakeArmSubsystem IntakeArmSubsystem) {
      m_IntakeArmSubsystem = IntakeArmSubsystem;

      addRequirements(IntakeArmSubsystem);
    }

    @Override
    public void initialize() {
        m_IntakeArmSubsystem.intakeArmExtend();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
       return m_IntakeArmSubsystem.isIntakeArmExtended();
    }
    
    @Override
    public void end(boolean interrupted) {
      m_IntakeArmSubsystem.stopIntakeArm();
    }
}



