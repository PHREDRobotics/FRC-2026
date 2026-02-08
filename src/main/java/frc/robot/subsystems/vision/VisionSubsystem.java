package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * Subsystem for controller all vision aspects of the robot
 */
public class VisionSubsystem extends SubsystemBase {
  private PhotonCamera m_camera;

  private Transform3d m_robotToTarget;

  private PhotonPipelineResult result = new PhotonPipelineResult();

  private PhotonPoseEstimator m_photonPoseEstimator;

  public VisionSubsystem() {
    m_camera = new PhotonCamera(VisionConstants.kCameraName);

    m_photonPoseEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout, VisionConstants.kRobotToCamera1);
  }

  /**
   * Returns true if a valid april tag is seen
   * @return
   */
  public boolean hasValidTarget() {
    if (result.hasTargets()) {
      return true;
    } else {
      return false;
    }
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

    Pose3d robotPose = cameraPose.transformBy(VisionConstants.kRobotToCamera1.inverse());
    SmartDashboard.putString("currentRelativePose", robotPose.toString());
    return Optional.ofNullable(robotPose.toPose2d());
  }

  @Override
  public void periodic() {
    var results = m_camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      result = results.get(results.size() - 1);

      if (result.hasTargets()) {
        m_robotToTarget = VisionConstants.kRobotToCamera1.plus(result.getBestTarget().getBestCameraToTarget());
        SmartDashboard.putNumber("robotToTarget/X", m_robotToTarget.getX());
        SmartDashboard.putNumber("robotToTarget/Y", m_robotToTarget.getY());
        SmartDashboard.putNumber("robotToTarget/Z", m_robotToTarget.getZ());
        SmartDashboard.putNumber("robotToTarget/Rot", m_robotToTarget.getRotation().toRotation2d().getRadians());

        SmartDashboard.putString("Estimated pose", getEstimatedGlobalPose().get().estimatedPose.toString());
      }
    }
  }
}