package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class ResetGyro {
  public static void resetGyro(PoseEstimatorSubsystem poseEstimatorSubsystem) {
    poseEstimatorSubsystem.setCurrentPose(new Pose2d(poseEstimatorSubsystem.getCurrentPose().getX(),
        poseEstimatorSubsystem.getCurrentPose().getY(), Rotation2d.fromDegrees(0)));
  }
}
