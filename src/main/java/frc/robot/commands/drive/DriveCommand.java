package frc.robot.commands.drive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * A command that allows the robot to be driven with a joystick.
 */
public class DriveCommand extends Command {
  private SwerveSubsystem m_swerveDrive;

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rotSpeed;
  private double m_throttle;
  private boolean m_fieldOriented;

  /**
   * Creates a new DriveCommand.
   * 
   * @param swerveDrive   The swerve subsystem
   * @param ySpeedFunc    Forward speed
   * @param xSpeedFunc    Sideways speed
   * @param rotFunc       Rotational speed
   * @param throttle      Speed control
   * @param fieldOriented Whether the robot should drive oriented to itself or the
   *                      field
   */
  public DriveCommand(
      SwerveSubsystem swerveDrive,
      DoubleSupplier x,
      DoubleSupplier y,
      DoubleSupplier rot,
      DoubleSupplier throttle,
      BooleanSupplier fieldOriented) {
    m_swerveDrive = swerveDrive;
    m_xSpeed = x.getAsDouble();
    m_ySpeed = y.getAsDouble();
    m_rotSpeed = rot.getAsDouble();
    m_throttle = throttle.getAsDouble();
    m_fieldOriented = fieldOriented.getAsBoolean();

    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {
    m_swerveDrive.drive(
        m_xSpeed,
        m_ySpeed,
        m_rotSpeed,
        m_fieldOriented);

    SmartDashboard.putNumber("Joystick/X", m_xSpeed);
    SmartDashboard.putNumber("Joystick/Y", m_ySpeed);
    SmartDashboard.putNumber("Joystick/ROT", m_rotSpeed);
    SmartDashboard.putNumber("Joystick/T", m_throttle);
  }
}