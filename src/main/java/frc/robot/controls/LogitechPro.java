package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LogitechPro {
  CommandJoystick m_joystick;

  public LogitechPro(int port) {
    m_joystick = new CommandJoystick(port);
  }

  public double getX() {
    return m_joystick.getX();
  }

  public double getY() {
    return m_joystick.getY();
  }

  public double getZ() {
    return m_joystick.getZ();
  }

  public double getThrottle() { // have to turn 1 -> -1 to 0 -> 1
    return (1 + (-1 * m_joystick.getThrottle())) / 2;
  }

  public boolean getSwerveReset() {
    return m_joystick.button(1).getAsBoolean();
  }

  public boolean getFieldOriented() {
    return m_joystick.button(0).getAsBoolean();
  }

  public Trigger getAlignTag() {
    return new Trigger(m_joystick.button(5));
  }

  public Trigger getFollowTag() {
    return new Trigger(m_joystick.button(4));
  }
}
