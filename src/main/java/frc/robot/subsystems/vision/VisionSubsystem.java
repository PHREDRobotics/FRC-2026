package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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

  private PhotonPoseEstimator m_photonPoseEstimator;

  /**
   * Creates a vision subsystem
   */
  public VisionSubsystem() {
    camera = new PhotonCamera(VisionConstants.kCameraName);

    robotToCamera = VisionConstants.robotToCamera1;

    m_photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, robotToCamera);
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
    return 0;
  }

  /**
   * Gets the estimated pose of the robot relative to the field
   * 
   * @return The estimated robot pose
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEst = m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
    if (visionEst.isEmpty()) {
      visionEst = m_photonPoseEstimator.estimateLowestAmbiguityPose(result);
    }

    return visionEst;
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

        SmartDashboard.putString("Estimated pose", getEstimatedGlobalPose().get().estimatedPose.toString());
      }
    }
  }
}