package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Configs for the spark maxes
 */
public final class Configs {
  public static final class FrontLeftConfig {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
          / Constants.SwerveConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(true);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)
          .velocityConversionFactor(drivingFactor / 60);
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(SwerveConstants.kDriveP, SwerveConstants.kDriveI, SwerveConstants.kDriveD)
          .outputRange(-1, 1);
      drivingConfig.closedLoop.feedForward.kV(0.0001);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(true);
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningFactor) // radiansd
          .velocityConversionFactor(turningFactor / 60);
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(SwerveConstants.kTurnP, SwerveConstants.kTurnI, SwerveConstants.kTurnD)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class FrontRightConfig {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
          / Constants.SwerveConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      double drivingVelocityFeedForward = 1 / 6;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(true);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)
          .velocityConversionFactor(drivingFactor / 60);
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(SwerveConstants.kDriveP, SwerveConstants.kDriveI, SwerveConstants.kDriveD)
          .outputRange(-1, 1);
      drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(true);
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningFactor) // radiansd
          .velocityConversionFactor(turningFactor / 60);
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(SwerveConstants.kTurnP, SwerveConstants.kTurnI, SwerveConstants.kTurnD)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class BackLeftConfig {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
          / Constants.SwerveConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      double drivingVelocityFeedForward = 1 / 6;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(true);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)
          .velocityConversionFactor(drivingFactor / 60);
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(SwerveConstants.kDriveP, SwerveConstants.kDriveI, SwerveConstants.kDriveD)
          .outputRange(-1, 1);
      drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(true);
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningFactor) // radiansd
          .velocityConversionFactor(turningFactor / 60);
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(SwerveConstants.kTurnP, SwerveConstants.kTurnI, SwerveConstants.kTurnD)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class BackRightConfig {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
          / Constants.SwerveConstants.kDrivingMotorReduction;
      double turningFactor = 2 * Math.PI;

      double drivingVelocityFeedForward = 1 / 6;

      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(true);
      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)
          .velocityConversionFactor(drivingFactor / 60);
      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(SwerveConstants.kDriveP, SwerveConstants.kDriveI, SwerveConstants.kDriveD)
          .outputRange(-1, 1);
      drivingConfig.closedLoop.feedForward.kV(drivingVelocityFeedForward);

      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20)
          .inverted(true);
      turningConfig.absoluteEncoder
          .inverted(true)
          .positionConversionFactor(turningFactor) // radiansd
          .velocityConversionFactor(turningFactor / 60);
      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          .pid(1, 0, 0)
          .outputRange(-1, 1)
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0, turningFactor);
    }
  }

  public static final class FeederConfig {
    public static final SparkMaxConfig feederMotorConfig = new SparkMaxConfig();
    static {
      feederMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(false);
    }
  }

  public static final class ShooterConfig {
    public static final SparkMaxConfig shooterMotorConfig = new SparkMaxConfig();
    static {
      shooterMotorConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50)
          .inverted(false);

      shooterMotorConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .pid(1, 0, 0);
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
          .outputRange(-1, 1);
    }
  }
}