package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.nio.file.Path;

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
    // public static final int kClimberMotorCANId = 6;

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

  /* -------------------- Hopper Constants ----------------------- */
  public static final class HopperConstants {

    // public static final int kHopperMotorCANId = 32;
    // public static final int kHopperVectorMotorCANId = 33;
    public static final double kHopperSpeedSetting = 0.5;

  }

  /* -------------------- Intake Constants ----------------------- */
  public static final class IntakeConstants {

    // public static final int kIntakeMotorCANId = 31;
    public static final double kIntakeSpeedSetting = 99; /* TODO */

  }

  /* -------------------- Intake Arm Constants ------------------- */
  public static final class IntakeArmConstants {

    // public static final int kIntakeArmMotorCANId = 51;

    public static final double kArmUpEncoderValue = 99; /* TODO */
    public static final double kArmDownEncoderValue = 99; /* TODO */

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

    public static final double kHubX = 3; // temp
    public static final double kHubY = 3; // temp
  }

  /* -------------------- Shooter Constants ---------------------- */
  public static final class ShooterConstants {
    public static final double kAutoShooterFactor = 0.1;

    public static final double kFeederSpeed = 0.5;

    // public static final int kFeederLeftMotorCANId = 36;
    // public static final int kFeederRightMotorCANId = 37;
    // public static final int kShooterFrontLeftMotorCANId = 41;
    // public static final int kShooterBackLeftMotorCANId = 42;
    // public static final int kShooterFrontRightMotorCANId = 46;
    // public static final int kShooterBackRightMotorCANId = 47;

    public static final double kAutoShooterDistanceMultiplier = 0.2;
    public static final double kAutoShooterDistanceExponent = 1.5;
  }

  /* -------------------- Swerve Constants ----------------------- */
  public static final class SwerveConstants {
    public static final double kWheelRadius = 0.0508;
    public static final int kEncoderResolution = 4096;

    public static final double kDtSeconds = 0.02;

    // public static final int kFrontLeftDriveMotorCANId = 11;
    // public static final int kFrontRightDriveMotorCANId = 16;
    // public static final int kBackLeftDriveMotorCANId = 21;
    // public static final int kBackRightDriveMotorCANId = 26;

    // public static final int kFrontLeftTurnMotorCANId = 12;
    // public static final int kFrontRightTurnMotorCANId = 17;
    // public static final int kBackLeftTurnMotorCANId = 22;
    // public static final int kBackRightTurnMotorCANId = 27;

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

    public static final double kXDeadband = 0.03;
    public static final double kYDeadband = 0.03;
    public static final double kRotDeadband = 0.05;
  }

  /* -------------------- Vision Constants ----------------------- */
  public static final class VisionConstants {
    public static final double kLimelightMountAngleDegrees = 0.0;
    public static final double kLimelightLensHeightInches = 0.0;
    public static final double kAreaToCentimeters = 150;
    public static final double kMetersFromAprilTag = 2;
    public static final double kMetersSideReefAprilTag = 6.47 * 0.0254; // the distance between prongs is
                                                                        // 12.94 so
                                                                        // divide by 2 and convert to
                                                                        // meters.
    public static final double kMetersFromReef = 8 * 0.0254; // 8 inches to meters
    public static final double kMetersFromLoadingStation = 14 * 0.0254; // 14 inches to meters

    public static final double kAprilTagGapMeters = 10.5 * 0.0254; // 10.5 inches to meters

    public static final double kPositionTolerance = 0.2;// TWEAK
    public static final double kAngleTolerance = 3;// TWEAK

    public static final String kCameraName = "ArducamOV9872 1";

    public static final double k2pi = Math.PI * 2;
    public static final AprilTagFieldLayout kAprilTagLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.k2026RebuiltWelded);

    public static final Pose2d kAprilTag1 = new Pose2d(657.37 * 0.0254, 25.80 * 0.0254,
        new Rotation2d(126 * k2pi / 360));
    public static final Pose2d kAprilTag2 = new Pose2d(657.37 * 0.0254, 291.20 * 0.0254,
        new Rotation2d(234 * k2pi / 360));
    public static final Pose2d kAprilTag3 = new Pose2d(455.15 * 0.0254, 317.15 * 0.0254,
        new Rotation2d(270 * k2pi / 360));
    public static final Pose2d kAprilTag4 = new Pose2d(365.20 * 0.0254, 241.64 * 0.0254,
        new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag5 = new Pose2d(365.20 * 0.0254, 75.39 * 0.0254,
        new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag6 = new Pose2d(530.49 * 0.0254, 130.17 * 0.0254,
        new Rotation2d(300 * k2pi / 360));
    public static final Pose2d kAprilTag7 = new Pose2d(546.87 * 0.0254, 158.50 * 0.0254,
        new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag8 = new Pose2d(530.49 * 0.0254, 186.83 * 0.0254,
        new Rotation2d(60 * k2pi / 360));
    public static final Pose2d kAprilTag9 = new Pose2d(497.77 * 0.0254, 186.83 * 0.0254,
        new Rotation2d(120 * k2pi / 360));
    public static final Pose2d kAprilTag10 = new Pose2d(481.39 * 0.0254, 158.50 * 0.0254,
        new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag11 = new Pose2d(497.77 * 0.0254, 130.17 * 0.0254,
        new Rotation2d(240 * k2pi / 360));

    // THE ONLY ACTUALLY IMPLEMENTED APRILTAG OF CURRENT DATE (2/8/2025, 10:32AM)
    public static final Pose2d kAprilTag12 = new Pose2d(33.51 * 0.0254, 25.80 * 0.0254,
        new Rotation2d(54 * k2pi / 360));
    public static final Pose2d kAprilTag13 = new Pose2d(33.51 * 0.0254, 291.20 * 0.0254,
        new Rotation2d(306 * k2pi / 360));
    public static final Pose2d kAprilTag14 = new Pose2d(325.68 * 0.0254, 241.64 * 0.0254,
        new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag15 = new Pose2d(325.68 * 0.0254, 75.39 * 0.0254,
        new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag16 = new Pose2d(235.73 * 0.0254, -0.15 * 0.0254,
        new Rotation2d(90 * k2pi / 360));
    public static final Pose2d kAprilTag17 = new Pose2d(160.39 * 0.0254, 130.17 * 0.0254,
        new Rotation2d(240 * k2pi / 360));
    public static final Pose2d kAprilTag18 = new Pose2d(144.00 * 0.0254, 158.50 * 0.0254,
        new Rotation2d(180 * k2pi / 360));
    public static final Pose2d kAprilTag19 = new Pose2d(160.39 * 0.0254, 186.83 * 0.0254,
        new Rotation2d(120 * k2pi / 360));
    public static final Pose2d kAprilTag20 = new Pose2d(193.10 * 0.0254, 186.83 * 0.0254,
        new Rotation2d(60 * k2pi / 360));
    public static final Pose2d kAprilTag21 = new Pose2d(209.49 * 0.0254, 158.50 * 0.0254,
        new Rotation2d(0 * k2pi / 360));
    public static final Pose2d kAprilTag22 = new Pose2d(193.10 * 0.0254, 130.17 * 0.0254,
        new Rotation2d(300 * k2pi / 360));

    public static final Pose2d[] kAprilTags = { kAprilTag1, kAprilTag2, kAprilTag3, kAprilTag4,
        kAprilTag5, kAprilTag6, kAprilTag7, kAprilTag8, kAprilTag9, kAprilTag10, kAprilTag11,
        kAprilTag12, kAprilTag13, kAprilTag14, kAprilTag15, kAprilTag16, kAprilTag17,
        kAprilTag18,
        kAprilTag19, kAprilTag20, kAprilTag21, kAprilTag22 };

    public static final Transform3d kRobotToCamera1 = new Transform3d(0, 0, 0, new Rotation3d());

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    // Camera facing forward. And offset to the robot center by half a meter up and
    // half a meter forward.
    public static final Transform3d robotToCamera1 = new Transform3d(0.5, 0, 0.5, new Rotation3d());
    public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
        .loadField(AprilTagFields.kDefaultField);

    public static final double kStrafeMult = 0.5;
    public static final Pose2d kOffset = new Pose2d(1, 0, new Rotation2d(Math.PI));
  }

}