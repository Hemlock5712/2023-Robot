package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.aStar.Pathfinding;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PPAStar extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;

  private Command otherCommand;

  private PathPlannerTrajectory trajectory;
  private final PathConstraints constraints;
  private List<Translation2d> internalPoints;
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
    var pose = poseEstimatorSystem.getCurrentPose();
    internalPoints = Pathfinding.generatePath((int) pose.getX(), (int) pose.getY(), (int) endX, (int) endY);
    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    if (internalPoints.size() > 0) {
      PathPoint[] restOfPoints = new PathPoint[internalPoints.size()];
      for (int i = 1; i < internalPoints.size(); i++) {
        restOfPoints[i - 1] = new PathPoint(internalPoints.get(i), Rotation2d.fromDegrees(endAngle),
            Rotation2d.fromDegrees(endAngle));
      }
      restOfPoints[restOfPoints.length - 1] = new PathPoint(new Translation2d(endX, endY),
          driveSystem.getGyroscopeRotation(), Rotation2d.fromDegrees(endAngle));

      trajectory = PathPlanner.generatePath(constraints,
          new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation()),
          new PathPoint(internalPoints.get(0), pose.getRotation(), pose.getRotation()),
          restOfPoints);
    } else {
      trajectory = PathPlanner.generatePath(constraints,
          new PathPoint(pose.getTranslation(), pose.getRotation(), pose.getRotation()),
          new PathPoint(new Translation2d(endX, endY), driveSystem.getGyroscopeRotation(),
              Rotation2d.fromDegrees(endAngle)));
    }

    poseEstimatorSystem.addTrajectory(trajectory);

    otherCommand = DrivetrainSubsystem.followTrajectory(driveSystem, poseEstimatorSystem, trajectory);
    otherCommand.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    otherCommand.cancel();
    driveSystem.stop();
  }
}
