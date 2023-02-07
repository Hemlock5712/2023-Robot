package frc.robot.commands;

import java.util.ArrayList;
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
    AStarMap.addNode(finalPosition);
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node endNode = AStarMap.getNode(i);
      AStarMap.addEdge(new Edge(finalPosition, endNode), obstacles);
    }
    addRequirements(driveSystem, poseEstimatorSystem);
  }

  // ----------------------------------------------------------------------------
  // Pre-schedule setup code.
  @Override
  public void initialize() {
    VisGraph tempGraph = AStarMap;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      startPoint = new Node(poseEstimatorSystem);
    } else {
      Pose2d flippedY = new Pose2d(poseEstimatorSystem.getCurrentPose().getX(),
          FieldConstants.fieldWidth - poseEstimatorSystem.getCurrentPose().getY(),
          poseEstimatorSystem.getCurrentPose().getRotation());
      startPoint = new Node(flippedY);
    }
    PathPlannerTrajectory trajectory;
    List<Node> fullPath = new ArrayList<Node>();

    tempGraph.addNode(startPoint);
    if (tempGraph.addEdge(new Edge(startPoint, finalPosition), obstacles)) {
      fullPath.add(0, startPoint);
      fullPath.add(1, finalPosition);
    } else {
      for (int i = 0; i < tempGraph.getNodeSize(); i++) {
        Node endNode = tempGraph.getNode(i);
        tempGraph.addEdge(new Edge(startPoint, endNode), obstacles);
      }
      fullPath = tempGraph.findPath(startPoint, finalPosition);
    }

    if (fullPath == null) {
      return;
    }

    // Gets speed of robot
    var chassisSpeeds = driveSystem.getChassisSpeeds();
    var fieldSpeeds = FieldOrientedDriveCommand.getFieldSpeeds(chassisSpeeds,
        poseEstimatorSystem.getCurrentPose().getRotation());
    var robotSpeeds = FieldOrientedDriveCommand.getRobotSpeeds(fieldSpeeds, chassisSpeeds);
    double startingSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
    Rotation2d heading = new Rotation2d(fullPath.get(1).getX() -
        startPoint.getX(),
        fullPath.get(1).getY() - startPoint.getY());
    // Rotation2d heading = new Rotation2d(robotSpeeds.vxMetersPerSecond,
    // robotSpeeds.vyMetersPerSecond);

    // If the robot is moving over a specified speed take movement into account.
    // if (startingSpeed > 0.05) {
    // heading = new Rotation2d(driveSystem.getChassisSpeeds().vxMetersPerSecond,
    // driveSystem.getChassisSpeeds().vyMetersPerSecond);
    // }

    // Depending on if internal points are present, make a new array of the other
    // points in the path.
    // PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
    ArrayList<PathPoint> fullPathPoints = new ArrayList<PathPoint>();
    // Find path between points
    for (int i = 0; i < fullPath.size(); i++) {
      if (i == 0) {
        fullPathPoints.add(new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
            startPoint.getHolRot(), startingSpeed));    
        addMidPoints(fullPathPoints, i, finalPosition.getHolRot());
      }

      else if (i + 1 == fullPath.size()) {
        heading = new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
            fullPath.get(i).getY() - fullPath.get(i - 1).getY());
        fullPathPoints.add(new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
            heading,
            finalPosition.getHolRot()));
      }

      else {
        // Change allianceFinal.getHolRot() to null if you want it to turn smoothly over
        // path. (Needs more testing)
        heading = new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
            fullPath.get(i + 1).getY() - fullPath.get(i).getY());
        fullPathPoints.add(new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
            heading,
            finalPosition.getHolRot()));
        addMidPoints(fullPathPoints, i, finalPosition.getHolRot());
      }
    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    // System.out.println(fullPathPoints);
    trajectory = PathPlanner.generatePath(constraints, fullPathPoints);
    // Display Trajectory
    poseEstimatorSystem.addTrajectory(trajectory);
    // Change trajectory based on alliance color
    trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
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

  public void addMidPoints(ArrayList<PathPoint> fullPath, int i, Rotation2d midPointHol){
    double distance = Math.hypot(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
        fullPath.get(i + 1).getY() - fullPath.get(i).getY());
    int midpoints = (int) Math.floor(distance / 4);
    Rotation2d heading = new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
      fullPath.get(i + 1).getY() - fullPath.get(i).getY());
    for (int j = 0; j < midpoints; j++) {
      fullPathPoints.add(new PathPoint(
          new Translation2d((fullPath.get(i + 1).getX() + fullPath.get(i).getX()) * ((j + 1) / (midpoints + 1)),
              (fullPath.get(i + 1).getY() + fullPath.get(i).getY()) * ((j + 1) / (midpoints + 1))),
          heading,
          midPointHol));
    }
  }
}
