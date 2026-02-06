package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * Constants for the robot
 */
public class Constants {

  /* -------------------- Climber Constants ------------------------ */
  public static final class ClimberConstants {
    public static final int kClimberMotorCANId = 6;

    public static final double kClimberExtendPower = 0.3; // this lowers the robot
    public static final double kClimberRetractPower = 1.0; // This raises the robot

    public static final double kClimberRaisedEncoderValue = 99; /* TODO */
    public static final double kClimberClimbedEncoderValue = 99; /* TODO */
    public static final double kClimberLoweredEncoderValue = 99; /* TODO */
  }

  /* -------------------- Controller Constants ------------------- */
  public static final class ControllerConstants {
    public static final double kFlightStickXDeadband = 0.2;
    public static final double kFlightStickYDeadband = 0.15;
    public static final double kFlightStickZDeadband = 0.15;

    public static final double kXRateLimit = 20;
    public static final double kYRateLimit = 20;
    public static final double kZRateLimit = 20;

    public static final double kXboxDeadband = 0.15;

    public static final double kThrottleMultiplier = 1;

    public static final double kMaxThrottle = 5;
    public static final double kMinThrottle = 2.5;
  }

  /* -------------------- Gyro Constants ------------------------- */
  public static final class GyroConstants {
    public static final NavXComType kComType = NavXComType.kMXP_SPI;
  }

  /* -------------------- Fuel Constants ----------------------- */
  // Intake, Hopper, and Vector
  public static final class FuelConstants {

    public static final int kHopperMotorCANId = 32;
    public static final int kIntakeMotorCANId = 31;
    public static final int kVectorMotorCANId = 33;

    public static final double kIntakeSpeedSetting = 99; /* TODO */

    public static final double kHopperSpeedSetting = 0.5;
    public static final double kVectorSpeedSetting = 0.5;

  }

  /* -------------------- Intake Arm Constants ------------------- */
  public static final class IntakeArmConstants {

    public static final int kIntakeArmMotorCANId = 51;

    public static final double kIntakeArmExtendPower = 0.5;
    public static final double kIntakeArmRetractPower = 0.5;

    public static final double kArmUpEncoderValue = 99; /* TODO */
    public static final double kArmDownEncoderValue = 99; /* TODO */

    public static final double kArmP = 0; // PID Tuning Values
    public static final double kArmD = 0;
    public static final double kArmI = 0;
  }

  /* -------------------- Physical Constants --------------------- */
  public static final class PhysicalConstants {
    public static final double kRobotMassPounds = 120;

    public static final double kBumperLength = 35.5;
    public static final double kTrackLength = 24;

    public static final double kMaxSpeed = 6;
    public static final double kMaxAngularSpeed = 6;

    public static final double kMaxAcceleration = 3;
    public static final double kMaxAngularAcceleration = 3;

    public static final double kNeoFreeSpeedRpm = 5676;
  }

  /* -------------------- Shooter Constants ---------------------- */
  public static final class ShooterConstants {
    public static final double kAutoShooterFactor = 0.1 * (Math.floor(Math.PI) / Math.round(Math.E));

    public static final double kFeederSpeed = 0.5;

    public static final int kFeederLeftMotorCANId = 36;
    public static final int kFeederRightMotorCANId = 37;

    
    public static final int kShooterLeftMotorCANId = 41;
    public static final int kShooterRightMotorCANId = 42;
    //public static final int kShooterBackLeftMotorCANId = 46;
    //public static final int kShooterBackRightMotorCANId = 47;

    public static final double kInitialShootingSpeed = 1000;

    public static final double kAutoShooterDistanceMultiplier = 0.2;
    public static final double kAutoShooterDistanceExponent = 1.5;

    public static final double kP = 0.0005;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFFV = 0.001;
  }

  /* -------------------- Swerve Constants ----------------------- */
  public static final class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kDtSeconds = 0.02;

    public static final int kFrontLeftDriveMotorCANId = 11;
    public static final int kFrontRightDriveMotorCANId = 16;
    public static final int kBackLeftDriveMotorCANId = 21;
    public static final int kBackRightDriveMotorCANId = 26;

    public static final int kFrontLeftTurnMotorCANId = 12;
    public static final int kFrontRightTurnMotorCANId = 17;
    public static final int kBackLeftTurnMotorCANId = 22;
    public static final int kBackRightTurnMotorCANId = 27;

    public static final double kDrivingMotorReduction = 8;
    public static final double kTurningMotorReduction = 21;

    public static final Translation2d kFrontLeftLocationInches = new Translation2d(12.25, 12.25);
    public static final Translation2d kFrontRightLocationInches = new Translation2d(12.25, -12.25);
    public static final Translation2d kBackLeftLocationInches = new Translation2d(-12.25, 12.25);
    public static final Translation2d kBackRightLocationInches = new Translation2d(-12.25, -12.25);

    public static final Translation2d kFrontLeftLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kFrontLeftLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kFrontLeftLocationInches.getY()));
    public static final Translation2d kFrontRightLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kFrontRightLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kFrontRightLocationInches.getY()));
    public static final Translation2d kBackLeftLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kBackLeftLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kBackLeftLocationInches.getY()));
    public static final Translation2d kBackRightLocationMeters = new Translation2d(
        Units.inchesToMeters(Constants.SwerveConstants.kBackRightLocationInches.getX()),
        Units.inchesToMeters(Constants.SwerveConstants.kBackRightLocationInches.getY()));

    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
        Constants.SwerveConstants.kFrontLeftLocationMeters,
        Constants.SwerveConstants.kFrontRightLocationMeters,
        Constants.SwerveConstants.kBackLeftLocationMeters,
        Constants.SwerveConstants.kBackRightLocationMeters);

    public static final Matrix<N3, N1> kStateStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
    public static final Matrix<N3, N1> kVisionStdDevs = VecBuilder.fill(0.8, 0.8, 0.8);

    public static final double kDriveP = 0.4;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveFF = 0.001;

    public static final double kTurnP = 1.0;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0.0;
    public static final double kTurnFF = 0.001;

    public static final double kXDeadband = 0.03;
    public static final double kYDeadband = 0.03;
    public static final double kRotDeadband = 0.05;
  }

  /* -------------------- Vision Constants ----------------------- */
  public static final class VisionConstants {
    public static final String kCameraName = "ArducamOV9872 1";

    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera facing forward. And offset to the robot center by half a meter up and
    // half a meter forward.
    public static final Transform3d robotToCamera1 = new Transform3d(0.5, 0, 0.5, new Rotation3d()); // CHANGEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE

    public static final Pose2d kOffset = new Pose2d(1, 0, new Rotation2d(Math.PI));

    public static final double kMetersFromAprilTag = 2;

    public static final double kXYPosP = 0.4;
    public static final double kXYPosI = 0.001;
    public static final double kXYPosD = 0.005;
    public static final TrapezoidProfile.Constraints kXYControllerConstraints = new TrapezoidProfile.Constraints(
        0.5,
        0.5);

    public static final double kRotP = 0.4;
    public static final double kRotI = 0.001;
    public static final double kRotD = 0.025;
    public static final TrapezoidProfile.Constraints kRotControllerConstraints = new TrapezoidProfile.Constraints(
        0.25,
        0.5);
  }
}