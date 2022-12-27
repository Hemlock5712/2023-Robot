package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.aStar.Pathfinding;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PPAStar extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private PPSwerveControllerCommand pathDrivingCommand;

  private PathPlannerTrajectory trajectory;
  private final PathConstraints constraints;
  public final double endX;
  public final double endY;
  public final double endAngle;

  public PPAStar(DrivetrainSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, double endX, double endY,
      double endAngle) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.constraints = constraints;
    this.endX = endX;
    this.endY = endY;
    this.endAngle = endAngle;

    addRequirements(driveSystem, poseEstimatorSystem);
  }

  @Override
  public void initialize() {
    Rotation2d endRotationObj = Rotation2d.fromDegrees(endAngle);
    Pose2d pose = poseEstimatorSystem.getCurrentPose();
    List<Translation2d> internalPoints = Pathfinding.generatePath((int) pose.getX(), (int) pose.getY(),
      (int) endX, (int) endY);
    
    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    if (internalPoints.size() > 0)
    {
      PathPoint[] restOfPoints = new PathPoint[internalPoints.size() - 1];

      PathPoint secondPoint = new PathPoint(internalPoints.get(0), internalPoints.get(1).minus(internalPoints.get(0)).getAngle(), endRotationObj);

      for (int i = 0; i < internalPoints.size() - 1; i++)
      {
        double y1 = internalPoints.get(i+1).getY();
        double x1 = internalPoints.get(i+1).getY();
        double y0 = internalPoints.get(i).getY();
        double x0 = internalPoints.get(i).getY();
        double hypotenuse = Math.hypot(y1 - y0, x1 - x0);
        Rotation2d angleOfTangentLineToXAxis = new Translation2d(x0 / hypotenuse, y0 / hypotenuse).getAngle();
        restOfPoints[i] = new PathPoint(internalPoints.get(i + 1), angleOfTangentLineToXAxis, endRotationObj);
      }
      restOfPoints[restOfPoints.length - 1] = new PathPoint(new Translation2d(endX, endY), endRotationObj, endRotationObj);

      trajectory = PathPlanner.generatePath(constraints,
          new PathPoint(pose.getTranslation(), internalPoints.get(0).minus(pose.getTranslation()).getAngle(), driveSystem.getModulePositions()[0].angle),
          secondPoint,
          restOfPoints);
    } else {
          trajectory = PathPlanner.generatePath(constraints,
          new PathPoint(pose.getTranslation(), internalPoints.get(0).minus(pose.getTranslation()).getAngle(), driveSystem.getModulePositions()[0].angle),
          new PathPoint(new Translation2d(endX, endY), endRotationObj, endRotationObj));
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
