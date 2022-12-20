package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class DriveWithPathPlanner extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;

  private PPSwerveControllerCommand pathDrivingCommand;

  private PathPlannerTrajectory trajectory;
  private final PathConstraints constraints;
  private final PathPoint finalPoint;
  private final PathPoint[] internalPoints;


  public DriveWithPathPlanner(DrivetrainSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, PathPoint finalPoint, PathPoint... internalPts) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.constraints = constraints;
    this.finalPoint = finalPoint;
    this.internalPoints = internalPts;

    addRequirements(driveSystem, poseEstimatorSystem);
  }

  @Override
  public void initialize()
  {
    var pose = poseEstimatorSystem.getCurrentPose();
    // Depending on if internal points are present, make a new array of the other points in the path.
    if (internalPoints.length > 0)
    {
      PathPoint[] restOfPoints = new PathPoint[internalPoints.length];
      for (int i = 1; i < internalPoints.length; i++)
      {
        restOfPoints[i - 1] = internalPoints[i];
      }
      restOfPoints[restOfPoints.length - 1] = finalPoint;

      trajectory = PathPlanner.generatePath(constraints, new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation()), internalPoints[0], restOfPoints);
    }
    else
    {
      trajectory = PathPlanner.generatePath(constraints, new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation()), finalPoint);
    }
    
    pathDrivingCommand = DrivetrainSubsystem.followTrajectory(driveSystem, poseEstimatorSystem, trajectory);
    pathDrivingCommand.schedule();
  }

  @Override
  public void end(boolean interrupted)
  {
    pathDrivingCommand.cancel();
    driveSystem.stop();
  }
}
