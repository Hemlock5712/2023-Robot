package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.pathfind.Edge;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
public class WPIAStar extends CommandBase {
  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private Command pathDrivingCommand;
  private final Node finalPosition;
  private Node startPoint;
  private final List<Obstacle> obstacles;
  private VisGraph AStarMap;
  private TrajectoryConfig config;
            // Add kinematics to ensure max speed is actually obeyed
            




  public WPIAStar(DrivetrainSubsystem d, PoseEstimatorSubsystem p, TrajectoryConfig config, Node finalPosition, List<Obstacle> obstacles, VisGraph AStarMap) {
    this.driveSystem = d;
    this.poseEstimatorSystem = p;
    this.config = config.setKinematics(DrivetrainConstants.KINEMATICS);
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
    config.setStartVelocity(Math.hypot(driveSystem.getChassisSpeeds().vxMetersPerSecond,driveSystem.getChassisSpeeds().vxMetersPerSecond));
    List<Node> fullPath = new ArrayList<Node>();
    startPoint = new Node(poseEstimatorSystem.getCurrentPose().getX(), poseEstimatorSystem.getCurrentPose().getY(), poseEstimatorSystem.getCurrentPose().getRotation()); 
    AStarMap.addNode(startPoint);
    if(AStarMap.addEdge(new Edge(startPoint, finalPosition), obstacles)){
      fullPath.add(0,startPoint);
      fullPath.add(1,finalPosition);
    }
    else{
      for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
        //System.out.println(String.format("SUCCESS: %f,%f %f,%f - %s", startPoint.getX(), startPoint.getY(), endNode.getX(), endNode.getY(), test ? "true" : "false"));
      }
      fullPath = AStarMap.findPath(startPoint, finalPosition);

    }
    

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    Translation2d[] fullPathPoints = new Translation2d[fullPath.size()-2];
    for(int i=1; i<fullPath.size()-1; i++){
        fullPathPoints[i-1] = new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY());
    }
    // Declare an array to hold PathPoint objects made from all other points specified in constructor.
  // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(startPoint.getX(), startPoint.getY(), startPoint.getHolRot()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(fullPathPoints),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(finalPosition.getX(), finalPosition.getY(), finalPosition.getHolRot()),
        // Pass config
        config);
    poseEstimatorSystem.addTrajectory(exampleTrajectory);
    pathDrivingCommand = driveSystem.createCommandForTrajectory(exampleTrajectory, poseEstimatorSystem::getCurrentPose);
    //RunCommand pathDrivingCommand = new RunCommand(driveSystem::stop, driveSystem);
    //pathDrivingCommand = DrivetrainSubsystem.followTrajectory(driveSystem, poseEstimatorSystem, trajectory);
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