package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PPAStar extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;

  private Command otherCommand;

  private PathPlannerTrajectory trajectory;
  private final PathConstraints constraints;
  private final PathPoint finalPoint;
  private final PathPoint[] internalPoints;


  public PPAStar(DrivetrainSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, PathPoint finalPoint, PathPoint... internalPts) {
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

    poseEstimatorSystem.addTrajectory(trajectory);
    
    otherCommand = DrivetrainSubsystem.followTrajectory(driveSystem, poseEstimatorSystem, trajectory);
    otherCommand.schedule();
  }

  @Override
  public void end(boolean interrupted)
  {
    otherCommand.cancel();
    driveSystem.stop();
  }
}
