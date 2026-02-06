// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.climber.ClimberSubsystem;

// /**
//  * Command for manually climb the robot
//  */

// public class ClimberClimbCommand extends Command {

//     private ClimberSubsystem m_climberSubsystem;

//     /**
//      * 
//      * @param climberSubsystem
//      */
//     public ClimberClimbCommand(ClimberSubsystem climberSubsystem ) {
//       m_climberSubsystem = climberSubsystem;

//       addRequirements(climberSubsystem);
//     }

//     @Override
//     public void initialize() {
//         m_climberSubsystem.startClimberRetract();
//     }

//     @Override
//     public void execute() {
//     }

//     @Override
//     public boolean isFinished() {
//        return m_climberSubsystem.isRobotClimbed();
//     }

//     @Override
//     public void end(boolean interrupted) {
//       m_climberSubsystem.stopClimber();
//     }
// }

