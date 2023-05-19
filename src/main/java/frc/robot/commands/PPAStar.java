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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
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
    AStarMap.addNode(finalPosition);
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node endNode = AStarMap.getNode(i);
      AStarMap.addEdge(new Edge(finalPosition, endNode), obstacles);
    }
    addRequirements(driveSystem);
  }

  // ----------------------------------------------------------------------------
  // Pre-schedule setup code.
  @Override
  public void initialize() {

    /*
     * Add starting position to map
     */

    // Create the starting position
    // Because of the map this year and we only want to create one field we need to
    // flip over the y axis
    if (Constants.DrivetrainConstants.alliance == Alliance.Blue) {
      // This is non flipped
      startPoint = new Node(poseEstimatorSystem.getCurrentPose());
    } else {
      // Flips of y axis
      Pose2d flippedY = new Pose2d(poseEstimatorSystem.getCurrentPose().getX(),
          FieldConstants.FIELD_WIDTH_METERS - poseEstimatorSystem.getCurrentPose().getY(),
          poseEstimatorSystem.getCurrentPose().getRotation());
      startPoint = new Node(flippedY);
    }

    // What will be our trajectory for path planner to follow
    PathPlannerTrajectory trajectory;
    List<Node> fullPath = new ArrayList<Node>();

    // Adds start point (current position on the field)
    AStarMap.addNode(startPoint);

    /*
     * START OF FINDING PATH
     */

    // Connects our starting point to all other nodes on the field (this does check
    // for obstacles)
    for (int i = 0; i < AStarMap.getNodeSize(); i++) {
      Node endNode = AStarMap.getNode(i);
      AStarMap.addEdge(new Edge(startPoint, endNode), obstacles);
    }
    // Finds the path using A*
    fullPath = AStarMap.findPath(startPoint, finalPosition);

    // If a path is not valid that is most likely because the starting point is
    // inside of an obstacle
    // Runs the same code as above only with smaller obstacles
    if (fullPath == null) {
      for (int i = 0; i < AStarMap.getNodeSize(); i++) {
        Node endNode = AStarMap.getNode(i);
        AStarMap.addEdge(new Edge(startPoint, endNode), FieldConstants.shortObstacles);
      }
      fullPath = AStarMap.findPath(startPoint, finalPosition);

      // Returns if no path is found
      if (fullPath == null) {
        return;
      }
    }

    /*
    * 
    */

    // Gets speed of robot
    var chassisSpeeds = driveSystem.getChassisSpeeds();
    var fieldSpeeds = FieldOrientedDriveCommand.getFieldSpeeds(chassisSpeeds,
        poseEstimatorSystem.getCurrentPose().getRotation());
    var robotSpeeds = FieldOrientedDriveCommand.getRobotSpeeds(fieldSpeeds, chassisSpeeds);
    double startingSpeed = Math.hypot(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

    // Gets heading that the robot needs to go to reach first point
    Rotation2d heading = new Rotation2d(fullPath.get(1).getX() -
        startPoint.getX(),
        fullPath.get(1).getY() - startPoint.getY());

    ArrayList<PathPoint> fullPathPoints = new ArrayList<PathPoint>();

    Rotation2d finalHol = finalPosition.getHolRot();

    /*
     * Takes our path found above and sends it to an ArrayList of Path Planner
     * Points. Also adds in midpoints
     * I would recommend commenting out addMidPoints at first when first testing
     * code
     */

    // Find path between points
    for (int i = 0; i < fullPath.size(); i++) {
      if (i == 0) {
        fullPathPoints.add(new PathPoint(new Translation2d(startPoint.getX(), startPoint.getY()), heading,
            startPoint.getHolRot(), startingSpeed));
        addMidPoints(fullPathPoints, fullPath, i, finalHol);
      }

      else if (i + 1 == fullPath.size()) {
        heading = new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
            fullPath.get(i).getY() - fullPath.get(i - 1).getY());
        fullPathPoints.add(new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
            heading,
            finalPosition.getHolRot()));
      }

      else {
        heading = new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
            fullPath.get(i + 1).getY() - fullPath.get(i).getY());
        fullPathPoints.add(new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
            heading,
            null));
        addMidPoints(fullPathPoints, fullPath, i, finalHol);
      }
    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    // System.out.println(fullPathPoints);
    trajectory = PathPlanner.generatePath(constraints, fullPathPoints);
    // Display Trajectory
    poseEstimatorSystem.addTrajectory(trajectory);
    // Change trajectory based on alliance color
    trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory,
        Constants.DrivetrainConstants.alliance);
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

  // Adds MidPoints so that Bezier curve doesn't curve into obstacle
  public void addMidPoints(ArrayList<PathPoint> fullPathPoints, List<Node> fullPath, int i, Rotation2d midPointHol) {
    double distance = Math.hypot(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
        fullPath.get(i + 1).getY() - fullPath.get(i).getY());

    // Adjust distance / x to have more or less midpoints lower the x the more
    // midpoints
    int midpoints = (int) Math.floor(distance / 1);

    Rotation2d heading = new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
        fullPath.get(i + 1).getY() - fullPath.get(i).getY());

    Rotation2d tempHol = null;
    for (int j = 0; j < midpoints; j++) {
      if (j % 2 == 0) {
        tempHol = null;
      } else {
        tempHol = midPointHol;
      }
      fullPathPoints.add(new PathPoint(
          new Translation2d(
              fullPath.get(i).getX()
                  + (fullPath.get(i + 1).getX() - fullPath.get(i).getX()) * ((j + 1.0) / (midpoints + 1.0)),
              fullPath.get(i).getY()
                  + (fullPath.get(i + 1).getY() - fullPath.get(i).getY()) * ((j + 1.0) / (midpoints + 1.0))),
          heading,
          tempHol));
    }
  }
}
