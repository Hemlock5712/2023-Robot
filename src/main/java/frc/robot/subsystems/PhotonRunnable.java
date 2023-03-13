package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD;
import static frc.robot.Constants.VisionConstants.ROBOT_TO_CAMERA;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.FieldConstants;

/**
 * Runnable that gets AprilTag data from PhotonVision.
 */
public class PhotonRunnable implements Runnable {

  private final PhotonPoseEstimator leftPoseEstimator;
  private final PhotonPoseEstimator rightPoseEstimator;

  private final PhotonCamera rightCamera;
  private final PhotonCamera leftCamera;
  private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
  private int rightScore = 0;
  private int leftScore = 0;

  public PhotonRunnable() {
    this.rightCamera = new PhotonCamera("photonvision");
    this.leftCamera = new PhotonCamera("leftCamera");
    PhotonPoseEstimator leftPhotonPoseEstimator = null;
    PhotonPoseEstimator rightPhotonPoseEstimator = null;
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      // PV estimates will always be blue, they'll get flipped by robot thread
      layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      if (rightCamera != null) {
        rightPhotonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP, rightCamera, ROBOT_TO_CAMERA);
        rightPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      }
      if (leftCamera != null) {
        leftPhotonPoseEstimator = new PhotonPoseEstimator(
            layout, PoseStrategy.MULTI_TAG_PNP, leftCamera, ROBOT_TO_CAMERA);
        leftPhotonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
      }
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      leftPhotonPoseEstimator = null;
      rightPhotonPoseEstimator = null;
    }
    this.leftPoseEstimator = leftPhotonPoseEstimator;
    this.rightPoseEstimator = rightPhotonPoseEstimator;
  }

  @Override
  public void run() {
    // Get AprilTag data
    if (rightPoseEstimator != null && rightCamera != null) {
      var rightResults = rightCamera.getLatestResult();
      var leftResults = leftCamera.getLatestResult();
      rightScore = 0;
      leftScore = 0;
      if (rightResults.hasTargets()
          && (rightResults.targets.size() > 1
              || rightResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        rightPoseEstimator.update(rightResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH_METERS) {
            if (rightResults.targets.size() < 2) {
              for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                Transform3d bestTarget = target.getBestCameraToTarget();
                double distance = Math.hypot(bestTarget.getX(), bestTarget.getY());
                if (distance < 4) {
                  rightScore = 1;
                }
              }
            } else {
              rightScore = 2;
            }
          }
        });
      }

      if (leftResults.hasTargets()
          && (leftResults.targets.size() > 1
              || leftResults.targets.get(0).getPoseAmbiguity() < APRILTAG_AMBIGUITY_THRESHOLD)) {
        leftPoseEstimator.update(leftResults).ifPresent(estimatedRobotPose -> {
          var estimatedPose = estimatedRobotPose.estimatedPose;
          // Make sure the measurement is on the field
          if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH_METERS
              && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH_METERS) {
            if (leftResults.targets.size() < 2) {
              for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
                Transform3d bestTarget = target.getBestCameraToTarget();
                double distance = Math.hypot(bestTarget.getX(), bestTarget.getY());
                if (distance < 4) {
                  leftScore = 1;
                }
              }
            } else {
              leftScore = 2;
            }
          }
        });
      }
      if (leftScore > 0 || rightScore > 0) {
        if (leftScore > rightScore) {
          atomicEstimatedRobotPose.set(leftPoseEstimator.update(leftResults).get());
        } else {
          atomicEstimatedRobotPose.set(rightPoseEstimator.update(rightResults).get());
        }
      }

      // if (rightResults.hasTargets()
      // && (rightResults.targets.size() > 1
      // || rightResults.targets.get(0).getPoseAmbiguity() <
      // APRILTAG_AMBIGUITY_THRESHOLD)) {
      // photonPoseEstimator.update(rightResults).ifPresent(estimatedRobotPose -> {
      // var estimatedPose = estimatedRobotPose.estimatedPose;
      // // Make sure the measurement is on the field
      // if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <=
      // FieldConstants.FIELD_LENGTH_METERS
      // && estimatedPose.getY() > 0.0 && estimatedPose.getY() <=
      // FieldConstants.FIELD_WIDTH_METERS) {
      // if (rightResults.targets.size() < 2) {
      // for (PhotonTrackedTarget target : estimatedRobotPose.targetsUsed) {
      // Transform3d bestTarget = target.getBestCameraToTarget();
      // double distance = Math.hypot(bestTarget.getX(), bestTarget.getY());
      // if (distance < 4) {
      // atomicEstimatedRobotPose.set(estimatedRobotPose);
      // }
      // }
      // } else {
      // atomicEstimatedRobotPose.set(estimatedRobotPose);
      // }
      // }
      // });
      // }
    }
  }

  /**
   * Gets the latest robot pose. Calling this will only return the pose once. If
   * it returns a non-null value, it is a
   * new estimate that hasn't been returned before.
   * This pose will always be for the BLUE alliance. It must be flipped if the
   * current alliance is RED.
   * 
   * @return latest estimated pose
   */
  public EstimatedRobotPose grabLatestEstimatedPose() {
    return atomicEstimatedRobotPose.getAndSet(null);
  }

}