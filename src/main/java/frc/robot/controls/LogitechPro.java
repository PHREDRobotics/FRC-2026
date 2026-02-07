package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants;

public class LogitechPro extends CommandJoystick {
  public LogitechPro(int port) {
    super(port);
  } 

  @Override
  public double getThrottle() { // have to turn 1 -> -1 to 0 -> 1
    return (1 + (-1 * super.getThrottle())) / 2;
  }

  public double getAdjustedThrottle() {
    return (getThrottle() - Constants.ControllerConstants.kMinThrottle) * (Constants.ControllerConstants.kMaxThrottle / (Constants.ControllerConstants.kMaxThrottle - Constants.ControllerConstants.kMinThrottle));
  }

  @Override
  public double getX() {
    SlewRateLimiter limiter = new SlewRateLimiter(Constants.ControllerConstants.kXRateLimit);

    double input = super.getX();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickXDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control
    input = limiter.calculate(input); // Limit acceleration

    return input;
  }

  @Override
  public double getY() {
    SlewRateLimiter limiter = new SlewRateLimiter(Constants.ControllerConstants.kYRateLimit);

    double input = super.getY();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickYDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control
    input = limiter.calculate(input); // Limit acceleration

    return input;
  }

  @Override
  public double getZ() {
    SlewRateLimiter limiter = new SlewRateLimiter(Constants.ControllerConstants.kZRateLimit);

    double input = super.getZ();

    input = MathUtil.applyDeadband(input, Constants.ControllerConstants.kFlightStickZDeadband); // Deadband
    input = input * Math.abs(input); // Square for better control
    input = limiter.calculate(input); // Limit acceleration

    return input;
  }
}
