package frc.robot.commands;

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
import frc.robot.PathFinder.AStar;
import frc.robot.PathFinder.Edge;
import frc.robot.PathFinder.Node;
import frc.robot.PathFinder.Obstacle;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class PPAStar extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private PPSwerveControllerCommand pathDrivingCommand;
  private final PathConstraints constraints;
  public final Node finalPosition;
  
  private AStar AStarMap;
  




  public PPAStar(DrivetrainSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, Node finalPosition, List<Obstacle> obstacles, AStar AStarMap) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.constraints = constraints;
    this.finalPosition = finalPosition;
    this.AStarMap = AStarMap;

    Node startPoint = new Node(p.getCurrentPose().getX(), p.getCurrentPose().getY());
    AStarMap.addNode(startPoint);
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
    }

    addRequirements(driveSystem, poseEstimatorSystem);
  }

  // ----------------------------------------------------------------------------
  // Per-schedule setup code.
  @Override
  public void initialize() 
  {
    PathPlannerTrajectory trajectory;

    List<Node> fullPath =  AStarMap.findPath(
      new Node(poseEstimatorSystem.getCurrentPose().getX(), 
      poseEstimatorSystem.getCurrentPose().getY()), 
      finalPosition);

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    PathPoint[] fullPathPoints = new PathPoint[fullPath.size() - 1];
    for(int i=0; i<fullPath.size(); i++){
      if(i<fullPath.size()-1){
        if(fullPath.get(i).getHolRot()==-1){
          fullPathPoints[i] = new PathPoint(new Translation2d(i, i), 
          new Rotation2d(fullPath.get(i+1).getX()-fullPath.get(i).getX(), 
          fullPath.get(i+1).getY()-fullPath.get(i).getY()));
        }
        else{
          fullPathPoints[i] = new PathPoint(new Translation2d(i, i), 
          new Rotation2d(fullPath.get(i+1).getX()-fullPath.get(i).getX(), 
          fullPath.get(i+1).getY()-fullPath.get(i).getY()),
          Rotation2d.fromDegrees(fullPath.get(i).getHolRot()));
        } 
      }
      else{
        if(fullPath.get(i).getHolRot()==-1){
          fullPathPoints[i] = new PathPoint(new Translation2d(i, i), 
          Rotation2d.fromDegrees(0),
          Rotation2d.fromDegrees(0));
        }
        else{
          fullPathPoints[i] = new PathPoint(new Translation2d(i, i), 
          Rotation2d.fromDegrees(fullPath.get(i).getHolRot()),
          Rotation2d.fromDegrees(fullPath.get(i).getHolRot()));
        } 
      }
      
    }
    // Declare an array to hold PathPoint objects made from all other points specified in constructor.
    if(fullPathPoints.length<3){
      trajectory = PathPlanner.generatePath(constraints, fullPathPoints[0], fullPathPoints[1]);
    }
    else{
      trajectory = PathPlanner.generatePath(constraints, fullPathPoints[0], fullPathPoints[1], Arrays.copyOfRange(fullPathPoints, 2, fullPathPoints.length));
    } 
    pathDrivingCommand = DrivetrainSubsystem.followTrajectory(driveSystem, poseEstimatorSystem, trajectory);
    pathDrivingCommand.schedule();
  }

  @Override
  public boolean isFinished()
  {
    return (pathDrivingCommand == null || !pathDrivingCommand.isScheduled());
  }

  @Override
  public void end(boolean interrupted)
  {
    if (interrupted)
    {
      pathDrivingCommand.cancel();
    }

    driveSystem.stop();
  }
}