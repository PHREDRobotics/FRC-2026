package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * Configs for the spark maxes
 */
public final class Configs {
  // public static final class SwerveConfig {
  //   public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
  //   public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

  //   static {
  //     double drivingFactor = Constants.SwerveConstants.kWheelRadius * 2 * Math.PI
  //         / Constants.SwerveConstants.kDrivingMotorReduction;
  //     double turningFactor = 2 * Math.PI;

  //     drivingConfig
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit(50)
  //         .inverted(true);
  //     drivingConfig.encoder
  //         .positionConversionFactor(drivingFactor)
  //         .velocityConversionFactor(drivingFactor / 60);
  //     drivingConfig.closedLoop
  //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
  //         .pid(0.4, 0, 0)
  //         .outputRange(-1, 1);

  //     turningConfig
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit(20)
  //         .inverted(true);
  //     turningConfig.absoluteEncoder
  //         .inverted(true)
  //         .positionConversionFactor(turningFactor) // radiansd
  //         .velocityConversionFactor(turningFactor / 60);
  //     turningConfig.closedLoop
  //         .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
  //         .pid(1, 0, 0)
  //         .outputRange(-1, 1)
  //         .positionWrappingEnabled(true)
  //         .positionWrappingInputRange(0, turningFactor);
  //   }
  // }

  // public static final class FeederMotorSparkMax {
  //   public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

  //   static {
  //     motorConfig
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit(50)
  //         .inverted(false);
  //   }
  // }

  // public static final class IntakeSparkMax {
  //   public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

  //   static {
  //     motorConfig
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit(50)
  //         .inverted(false);
  //   }
  // }

  // public static final class IntakeMovementSparkMax {
  //   public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

  //   static {
  //     motorConfig
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit(50)
  //         .inverted(false);
  //   }
  // }

  // public static final class HopperSparkMax {
  //   public static final SparkMaxConfig motorConfig = new SparkMaxConfig();

  //   static {
  //     motorConfig
  //         .idleMode(IdleMode.kBrake)
  //         .smartCurrentLimit(50)
  //         .inverted(false);
  //   }
  // }

}