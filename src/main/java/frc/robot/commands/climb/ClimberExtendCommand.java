package frc.robot.commands.climb;
// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.climber.ClimberSubsystem;

// /**
//  * Command for manually climb the robot
//  */

// public class ClimberExtendCommand extends Command {

//     private ClimberSubsystem m_climberSubsystem;

//     /**
//      * 
//      * @param climberSubsystem
//      */
//     public ClimberExtendCommand(ClimberSubsystem climberSubsystem ) {
//       m_climberSubsystem = climberSubsystem;

//       addRequirements(climberSubsystem);
//     }

//     @Override
//     public void initialize() {
//         m_climberSubsystem.startClimberExtend();
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public boolean isFinished() {
//        return m_climberSubsystem.isClimberExtended();
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//       m_climberSubsystem.stopClimber();
//     }
// }

