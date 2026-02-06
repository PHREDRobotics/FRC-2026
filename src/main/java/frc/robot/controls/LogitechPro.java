package frc.robot.controls;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class LogitechPro extends CommandJoystick {
  public LogitechPro(int port) {
    super(port);
  } 

  @Override
  public double getThrottle() { // have to turn 1 -> -1 to 0 -> 1
    return (1 + (-1 * super.getThrottle())) / 2;
  }
}
