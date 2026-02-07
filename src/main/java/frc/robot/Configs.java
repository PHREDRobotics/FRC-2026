package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Configs for the spark maxes
 */
public final class Configs {
  public static class SwerveConfig {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(true);
      drivingConfig.encoder
          .velocityConversionFactor(Constants.SwerveConstants.drivingFactor);
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(Constants.SwerveConstants.kDriveP, Constants.SwerveConstants.kDriveI, Constants.SwerveConstants.kDriveD)
          .outputRange(-1, 1);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(true);
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(Constants.SwerveConstants.turningFactor);
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(Constants.SwerveConstants.kTurnP, Constants.SwerveConstants.kTurnI, Constants.SwerveConstants.kTurnD)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, Math.PI * 2);
    }
  }

  public static final class FrontLeftConfig extends SwerveConfig {
    static {
    }
  }

  public static final class FrontRightConfig extends SwerveConfig {
    static {
    }
  }

  public static final class BackLeftConfig extends SwerveConfig {
    static {
      // Configs
    }
  }

  public static final class BackRightConfig extends SwerveConfig {
    static {
      // Configs
    }
  }

  public static final class FeederConfig {
    public static final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
    static {
      feederMotorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .inverted(false);
    }
  }

  public static final class ShooterConfig {
    public static final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
    static {
      shooterMotorConfig
          .idleMode(IdleMode.kCoast)
          .smartCurrentLimit(50)
          .inverted(false);

      shooterMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(Constants.ShooterConstants.kP, Constants.ShooterConstants.kI, Constants.ShooterConstants.kD).feedForward
          .kV(Constants.ShooterConstants.kFFV);
    }
  }

  public static final class IntakeArmConfig {
    public static final SparkMaxConfig intakeArmMotorConfig = new SparkMaxConfig();
    static {
      intakeArmMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Set PID values for position control. We don't need to pass a closed loop
          // slot, as it will default to slot 0.
          .p(Constants.IntakeArmConstants.kArmP)
          .i(Constants.IntakeArmConstants.kArmI)
          .d(Constants.IntakeArmConstants.kArmD)
          .outputRange(Constants.IntakeArmConstants.kArmDownEncoderValue, Constants.IntakeArmConstants.kArmUpEncoderValue);

      intakeArmMotorConfig.closedLoop.feedForward
          .kCos(Constants.IntakeArmConstants.kArmCos);
    }
  }
}