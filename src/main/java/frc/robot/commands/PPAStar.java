package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.pathfind.Edge;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PPAStar extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private PPSwerveControllerCommand pathDrivingCommand;
  private final PathConstraints constraints;
  private final Node finalPosition;
  private Node startPoint;
  private final List<Obstacle> obstacles;
  private VisGraph AStarMap;

  public PPAStar(DrivetrainSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, Node finalPosition,
      List<Obstacle> obstacles, VisGraph AStarMap) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.constraints = constraints;
    this.obstacles = obstacles;
    this.finalPosition = finalPosition;
    this.AStarMap = AStarMap;
    this.startPoint = new Node(p);

    addRequirements(driveSystem, poseEstimatorSystem);
  }

  // ----------------------------------------------------------------------------
  // Per-schedule setup code.
  @Override
  public void initialize() {
    startPoint = new Node(poseEstimatorSystem);
    PathPlannerTrajectory trajectory;
    List<Node> fullPath = new ArrayList<Node>();

    AStarMap.addNode(startPoint);
    if (AStarMap.addEdge(new Edge(startPoint, finalPosition), obstacles)) {
      fullPath.add(0, startPoint);
      fullPath.add(1, finalPosition);
    } else {
      for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
      }
      fullPath = AStarMap.findPath(startPoint, finalPosition);
    }
    
    if(fullPath == null){
      return;
    }

    Rotation2d Heading = new Rotation2d(fullPath.get(1).getX()-startPoint.getX(),fullPath.get(1).getY()-startPoint.getY());
    double totalDis = 0;
    for (int i = 0; i < fullPath.size() - 1; i++) {
      totalDis += Math.hypot(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
          fullPath.get(i + 1).getY() - fullPath.get(i).getY());
    }

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
    double distanceTraveled = 0;
    double currentPathDist = 0;
    for (int i = 0; i < fullPath.size(); i++) {
      if (i == 0) {
        fullPathPoints[i] = new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), Heading,
            poseEstimatorSystem.getCurrentPose().getRotation());
      } else if (i + 1 == fullPath.size()) {
        fullPathPoints[i] = new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
            new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
            finalPosition.getHolRot());
      } else {
        currentPathDist = Math.hypot(fullPath.get(i).getX() - fullPath.get(i-1).getX(), fullPath.get(i).getY() - fullPath.get(i-1).getY());
        distanceTraveled += currentPathDist;
        // fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
        //     new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
        //     Rotation2d.fromDegrees(
        //       angleAtPercent(poseEstimatorSystem.getCurrentPose().getRotation().getDegrees(), 
        //       finalPosition.getHolRot().getDegrees(), 
        //       distanceTraveled/totalDis)));
        fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
        new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
        finalPosition.getHolRot());
        // fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
        // new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
        // (Rotation2d)null);
      }

    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
    poseEstimatorSystem.addTrajectory(trajectory);
    pathDrivingCommand = DrivetrainSubsystem.followTrajectory(driveSystem, poseEstimatorSystem, trajectory);
    pathDrivingCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return (pathDrivingCommand == null || !pathDrivingCommand.isScheduled());
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      pathDrivingCommand.cancel();
    }

    driveSystem.stop();
  }
  
  public static double angleAtPercent(double start, double end, double percent) {
    double angleDiff = end - start;
    if (angleDiff > 180) {
        angleDiff -= 360;
    } else if (angleDiff < -180) {
        angleDiff += 360;
    }
    double angle = start + (angleDiff * percent);
    if (angle > 180) {
        angle -= 360;
    } else if (angle < -180) {
        angle += 360;
    }
    return angle;
  }
}
