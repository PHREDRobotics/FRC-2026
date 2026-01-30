// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.swerve.SwerveSubsystem;

// /**
//  * Resets the swerve odometry
//  */
// public class SwerveResetCommand extends Command {
//   private SwerveSubsystem m_swerveSubsystem;

//   /**
//    * Creates a new swerve reset command
//    * 
//    * @param swerveSubsystem the swerve subsystem to reset
//    */
//   public SwerveResetCommand(SwerveSubsystem swerveSubsystem) {
//     m_swerveSubsystem = swerveSubsystem;
//   }

//   @Override
//   public void initialize() {
//     m_swerveSubsystem.resetGyro();
//   }
// }