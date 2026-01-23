package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.proto.Photon;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera camera;

  private Transform3d robotToCamera;
  private Transform3d robotToTarget;

  private PhotonPipelineResult result = new PhotonPipelineResult();

  /**
   * Creates a vision subsystem
   */
  public VisionSubsystem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);

    robotToCamera = VisionConstants.robotToCamera1;
  }

  public Pose2d getTargetPose(Pose2d tag) {
    Pose2d targetPose = new Pose2d(
        tag.getX()
            + VisionConstants.kMetersFromAprilTag * Math.cos(tag.getRotation().getRadians()),
        tag.getY()
            + VisionConstants.kMetersFromAprilTag * Math.sin(tag.getRotation().getRadians()),
        tag.getRotation().rotateBy(new Rotation2d(Math.PI)));

    return targetPose;
  }

  public boolean hasValidTarget() {
    if (result.hasTargets()) {
      return true;
    } else {
      return false;
    }
  }

  public Transform3d getTagPose() {
    if (!result.hasTargets()) {
      return null;
    }
    Transform3d tagTransform = result.getBestTarget().getBestCameraToTarget();
    return tagTransform;
  }

  public double getHubDistance() {
    return 0; // TODO write the function
  }

  /**
   * Gets the estimated pose of the robot relative to the field
   * 
   * @return The estimated robot pose
   */
  public Optional<Pose2d> getEstimatedGlobalPose() {

    // If there are no targets, return an empty optional
    if (result.hasTargets() == false) {
      return Optional.empty();

    }
    // If there is one target, use that target to estimate the pose and only that
    // target
    else if (result.targets.size() == 1) {

      PhotonTrackedTarget target = result.getBestTarget();

      int id = target.getFiducialId();
      var tagPoseOpt = Constants.VisionConstants.kAprilTagLayout.getTagPose(id);

      Pose3d tagPose = tagPoseOpt.get();
      Transform3d cameraToTag = target.getBestCameraToTarget();
      Transform3d tagToCamera = cameraToTag.inverse();
      Pose3d cameraPose = tagPose.transformBy(tagToCamera);
      Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());

      return Optional.ofNullable(robotPose.toPose2d());
    }
    // If there are two or more targets, use the two best targets to estimate the
    // pose
    else {
      PhotonTrackedTarget target = result.getBestTarget();
      // Remove the best target to get the second best target
      
      int bestId = target.getFiducialId();

      result.targets.removeIf(t -> t.getFiducialId() == target.getFiducialId());
      PhotonTrackedTarget secondTarget = result.getBestTarget();

      int secondId = secondTarget.getFiducialId();

      var bestTagPoseOpt = Constants.VisionConstants.kAprilTagLayout.getTagPose(bestId);
      var secondTagPoseOpt = Constants.VisionConstants.kAprilTagLayout.getTagPose(secondId);

      Pose3d tagPose = bestTagPoseOpt.get();
      Pose3d secondTagPose = secondTagPoseOpt.get();

      Transform3d cameraToBestTag = target.getBestCameraToTarget();
      Transform3d cameraToSecondTag = secondTarget.getBestCameraToTarget();

      Transform3d tagToBestCamera = cameraToBestTag.inverse();
      Transform3d tagToSecondCamera = cameraToSecondTag.inverse();

      Pose3d cameraPoseFromBestTag = tagPose.transformBy(tagToBestCamera);
      Pose3d cameraPoseFromSecondTag = secondTagPose.transformBy(tagToSecondCamera);

      Pose3d robotPoseToBestTag = cameraPoseFromBestTag.transformBy(robotToCamera.inverse());
      Pose3d robotPoseToSecondTag = cameraPoseFromSecondTag.transformBy(robotToCamera.inverse());

      // The distance from the robot to the best tag
      double distanceToBestTag = Math.sqrt(
          Math.pow(robotPoseToBestTag.getX(), 2) +
              Math.pow(robotPoseToBestTag.getY(), 2));

      // The distance from the robot to the second tag
      double distanceToSecondTag = Math.sqrt(
          Math.pow(robotPoseToSecondTag.getX(), 2) +
              Math.pow(robotPoseToSecondTag.getY(), 2));

      // Using the law of cosines to estimate the distance to the best tag
      double estimatedDistanceToBestTag = Math.sqrt(
          Math.pow(Constants.VisionConstants.kAprilTagGapMeters, 2) +
              Math.pow(distanceToSecondTag, 2) -
              2 * Constants.VisionConstants.kAprilTagGapMeters *
                  distanceToSecondTag *
                  Math.cos(robotPoseToSecondTag.getRotation().toRotation2d().getRadians()));

      // Using the law of cosines to estimate the distance to the second best tag
      double estimatedDistanceToSecondTag = Math.sqrt(
          Math.pow(Constants.VisionConstants.kAprilTagGapMeters, 2) +
              Math.pow(distanceToBestTag, 2) -
              2 * Constants.VisionConstants.kAprilTagGapMeters *
                  distanceToSecondTag *
                  Math.cos(robotPoseToBestTag.getRotation().toRotation2d().getRadians()));

      // The estimated pose from the best tag
      Pose3d estimatedPoseFromBestTag = new Pose3d(
          estimatedDistanceToBestTag * Math.cos(robotPoseToBestTag.getRotation().toRotation2d().getRadians()),
          estimatedDistanceToBestTag * Math.sin(robotPoseToBestTag.getRotation().toRotation2d().getRadians()),
          0,
          robotPoseToBestTag.getRotation());

      // The estimated pose from the second best tag
      Pose3d estimatedPoseFromSecondTag = new Pose3d(
          estimatedDistanceToSecondTag * Math.cos(robotPoseToSecondTag.getRotation().toRotation2d().getRadians()),
          estimatedDistanceToSecondTag * Math.sin(robotPoseToSecondTag.getRotation().toRotation2d().getRadians()),
          0,
          robotPoseToSecondTag.getRotation());

      // Find the average of the two estimated positions
      double avgX = (estimatedPoseFromBestTag.getX() + estimatedPoseFromSecondTag.getX()) / 2;
      double avgY = (estimatedPoseFromBestTag.getY() + estimatedPoseFromSecondTag.getY()) / 2;
      double avgRotationRadians = (estimatedPoseFromBestTag.getRotation().toRotation2d().getRadians()
          + estimatedPoseFromSecondTag.getRotation().toRotation2d().getRadians()) / 2;

      // The actual average rotation
      Rotation2d avgRotation = new Rotation2d(avgRotationRadians);

      Pose3d estimatedPoseFromBothTags = new Pose3d(
          avgX,
          avgY,
          0,
          new Rotation3d(avgRotation));

      return Optional.ofNullable(estimatedPoseFromBothTags.toPose2d());
    }
  }

  public Optional<Pose2d> getEstimatedRelativePose() {
    PhotonTrackedTarget target = result.getBestTarget();
    Transform3d cameraToTag = target.getBestCameraToTarget();
    Transform3d tagToCamera = cameraToTag.inverse();
    Pose3d cameraPose = new Pose3d(tagToCamera.getX(), tagToCamera.getY(), tagToCamera.getZ(),
        tagToCamera.getRotation());

    Pose3d robotPose = cameraPose.transformBy(robotToCamera.inverse());
    SmartDashboard.putString("currentRelativePose", robotPose.toString());
    return Optional.ofNullable(robotPose.toPose2d());
  }

  @Override
  public void periodic() {
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        robotToTarget = robotToCamera.plus(result.getBestTarget().getBestCameraToTarget());
        SmartDashboard.putNumber("robotToTarget/X", robotToTarget.getX());
        SmartDashboard.putNumber("robotToTarget/Y", robotToTarget.getY());
        SmartDashboard.putNumber("robotToTarget/Z", robotToTarget.getZ());
        SmartDashboard.putNumber("robotToTarget/Rot", robotToTarget.getRotation().toRotation2d().getRadians());

        SmartDashboard.putString("Estimated pose", getEstimatedGlobalPose().get().toString());
      }
    }
  }
}