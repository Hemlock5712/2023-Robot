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
import frc.robot.pathfind.NavigationMesh;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
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
  private NavigationMesh AStarMap;
  




  public PPAStar(DrivetrainSubsystem d, PoseEstimatorSubsystem p, PathConstraints constraints, Node finalPosition, List<Obstacle> obstacles, NavigationMesh AStarMap) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.constraints = constraints;
    this.obstacles = obstacles;
    this.finalPosition = finalPosition;
    this.AStarMap = AStarMap;
    this.startPoint = new Node(p.getCurrentPose().getX(), p.getCurrentPose().getY(), p.getCurrentPose().getRotation());
    

    addRequirements(driveSystem, poseEstimatorSystem);
  }

  // ----------------------------------------------------------------------------
  // Per-schedule setup code.
  @Override
  public void initialize() 
  {
    startPoint = new Node(poseEstimatorSystem.getCurrentPose().getX(), poseEstimatorSystem.getCurrentPose().getY(), poseEstimatorSystem.getCurrentPose().getRotation());
    PathPlannerTrajectory trajectory;
    List<Node> fullPath = new ArrayList<Node>();

    AStarMap.addNode(startPoint);
   /* if(AStarMap.addEdge(new Edge(startPoint, finalPosition), obstacles)){
      fullPath.add(0,startPoint);
      fullPath.add(1,finalPosition);
    }
    else{*/
      for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
      }
      fullPath =  AStarMap.findPath(startPoint, finalPosition);
   // }
    

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    PathPoint[] fullPathPoints = new PathPoint[fullPath.size()-1];
    int pathSize = fullPath.size()-1;
    for(int i=1; i<pathSize; i++){
        fullPathPoints[i-1] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()), 
        new Rotation2d(fullPath.get(i+1).getX()-fullPath.get(i).getX(), 
        fullPath.get(i+1).getY()-fullPath.get(i).getY()));
    }
    
    fullPathPoints[pathSize] = new PathPoint(new Translation2d(fullPath.get(pathSize).getX(), fullPath.get(pathSize).getY()), 
      fullPath.get(pathSize).getHolRot(),
      fullPath.get(pathSize).getHolRot());
    // Declare an array to hold PathPoint objects made from all other points specified in constructor.
    trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
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