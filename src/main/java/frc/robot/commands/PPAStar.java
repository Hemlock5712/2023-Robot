package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.pathfind.Edge;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.FieldConstants;

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
    Node allianceFinal = finalPosition;
    if(DriverStation.getAlliance() == Alliance.Blue){
      startPoint = new Node(poseEstimatorSystem);
    }
    else{
      Pose2d flippedY = new Pose2d(poseEstimatorSystem.getCurrentPose().getX(),FieldConstants.fieldWidth-poseEstimatorSystem.getCurrentPose().getY(),poseEstimatorSystem.getCurrentPose().getRotation());
      allianceFinal = new Node(finalPosition.getX(),FieldConstants.fieldWidth-finalPosition.getY(), finalPosition.getHolRot());
      startPoint = new Node(flippedY);
    }
    PathPlannerTrajectory trajectory;
    List<Node> fullPath = new ArrayList<Node>();

    AStarMap.addNode(startPoint);
    AStarMap.addNode(allianceFinal);
    if (AStarMap.addEdge(new Edge(startPoint, allianceFinal), obstacles)) {
      fullPath.add(0, startPoint);
      fullPath.add(1, allianceFinal);
    } else {
      for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
      }
      for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(allianceFinal, endNode), obstacles);
      }
      fullPath = AStarMap.findPath(startPoint, allianceFinal);
    }
    
    if(fullPath == null){
      return;
    }
    
    double startingSpeed = Math.hypot(driveSystem.getChassisSpeeds().vxMetersPerSecond, driveSystem.getChassisSpeeds().vyMetersPerSecond);
    Rotation2d heading = new Rotation2d(fullPath.get(1).getX()-startPoint.getX(),fullPath.get(1).getY()-startPoint.getY());
    if(startingSpeed>0.05){
      heading = new Rotation2d(driveSystem.getChassisSpeeds().vxMetersPerSecond, driveSystem.getChassisSpeeds().vyMetersPerSecond);
    }

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
 
    for (int i = 0; i < fullPath.size(); i++) {
      if (i == 0) {
        fullPathPoints[i] = new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
            poseEstimatorSystem.getCurrentPose().getRotation(), startingSpeed);
      } else if (i + 1 == fullPath.size()) {
        fullPathPoints[i] = new PathPoint(new Translation2d(allianceFinal.getX(), allianceFinal.getY()),
            new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
            allianceFinal.getHolRot());
      } else {
        fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
        new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
        allianceFinal.getHolRot());
      }
    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
    //var alliance = Alliance.Blue;
    //trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);
    trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
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
}
