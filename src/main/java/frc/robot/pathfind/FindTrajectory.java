// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.pathfind;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.DrivetrainSubsystem;

/** Add your docs here. */
public class FindTrajectory {

  public static PathPlannerTrajectory getPathTrajectory(DrivetrainSubsystem drivetrain, PathConstraints constraints,
      List<Obstacle> obstacles, VisGraph aStarMap, Node startingPoint, Node finalPosition) {


        aStarMap.addNode(startingPoint);
        aStarMap.addNode(finalPosition);

        for (int i = 0; i < aStarMap.getNodeSize(); i++) {
          Node endNode = aStarMap.getNode(i);
          aStarMap.addEdge(new Edge(finalPosition, endNode), obstacles);
        }

        for (int i = 0; i < aStarMap.getNodeSize(); i++) {
          Node endNode = aStarMap.getNode(i);
          aStarMap.addEdge(new Edge(startingPoint, endNode), obstacles);
        }

        PathPlannerTrajectory trajectory;
        List<Node> fullPath = new ArrayList<Node>();
    

        fullPath = aStarMap.findPath(startingPoint, finalPosition);

    
        if (fullPath == null) {
          return null;
        }
    
        // Gets speed of robot
        double startingSpeed = Math.hypot(drivetrain.getChassisSpeeds().vxMetersPerSecond,
          drivetrain.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d heading = new Rotation2d(fullPath.get(1).getX() - startingPoint.getX(),
            fullPath.get(1).getY() - startingPoint.getY());
    
        // If the robot is moving over a specified speed take movement into account.
        if (startingSpeed > 0.05) {
          heading = new Rotation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond,
          drivetrain.getChassisSpeeds().vyMetersPerSecond);
        }
    
        // Depending on if internal points are present, make a new array of the other
        // points in the path.
        PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
    
        // Find path between points
        for (int i = 0; i < fullPath.size(); i++) {
          if (i == 0) {
            fullPathPoints[i] = new PathPoint(new Translation2d(startingPoint.getX(), startingPoint.getY()), heading,
            startingPoint.getHolRot());
          } else if (i + 1 == fullPath.size()) {
            fullPathPoints[i] = new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
                new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
                    fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
                    Rotation2d.fromDegrees(180));
          } else {
            // Change allianceFinal.getHolRot() to null if you want it to turn smoothly over
            // path. (Needs more testing)
            fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
                new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
                    fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
                    Rotation2d.fromDegrees(180));
          }
        }
    
        // Declare an array to hold PathPoint objects made from all other points
        // specified in constructor.
        trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
        // Change trajectory based on alliance color
        return PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
  }

  public static PathPlannerTrajectory getPathTrajectoryAuto(DrivetrainSubsystem drivetrain, PathConstraints constraints,
      List<Obstacle> obstacles, VisGraph aStarMap, Node startingPoint, Node finalPosition) {

        
        PathPlannerTrajectory trajectory;
        List<Node> fullPath = new ArrayList<Node>();
    
        aStarMap.addNode(startingPoint);
        aStarMap.addNode(finalPosition);

        for (int i = 0; i < aStarMap.getNodeSize(); i++) {
          Node endNode = aStarMap.getNode(i);
          aStarMap.addEdge(new Edge(finalPosition, endNode), obstacles);
        }

        for (int i = 0; i < aStarMap.getNodeSize(); i++) {
          Node endNode = aStarMap.getNode(i);
          aStarMap.addEdge(new Edge(startingPoint, endNode), obstacles);
        }

        fullPath = aStarMap.findPath(startingPoint, finalPosition);
    
        if (fullPath == null) {
          return null;
        }
    
        // Gets speed of robot
        double startingSpeed = Math.hypot(drivetrain.getChassisSpeeds().vxMetersPerSecond,
          drivetrain.getChassisSpeeds().vyMetersPerSecond);
        Rotation2d heading = new Rotation2d(fullPath.get(1).getX() - startingPoint.getX(),
            fullPath.get(1).getY() - startingPoint.getY());
    
        // If the robot is moving over a specified speed take movement into account.
        if (startingSpeed > 0.05) {
          heading = new Rotation2d(drivetrain.getChassisSpeeds().vxMetersPerSecond,
          drivetrain.getChassisSpeeds().vyMetersPerSecond);
        }
    
        // Depending on if internal points are present, make a new array of the other
        // points in the path.
        PathPoint[] fullPathPoints = new PathPoint[fullPath.size()];
    
        // Find path between points
        for (int i = 0; i < fullPath.size(); i++) {
          if (i == 0) {
            fullPathPoints[i] = new PathPoint(new Translation2d(startingPoint.getX(), startingPoint.getY()), heading,
            startingPoint.getHolRot());
          } else if (i + 1 == fullPath.size()) {
            fullPathPoints[i] = new PathPoint(new Translation2d(finalPosition.getX(), finalPosition.getY()),
                new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(),
                    fullPath.get(i).getY() - fullPath.get(i - 1).getY()), Rotation2d.fromDegrees(180)
                );

                //new Rotation2d((fullPath.get(i).getX() - fullPath.get(i - 1).getX()),
                //(fullPath.get(i).getY() - fullPath.get(i - 1).getY()))

          } else {
            // Change allianceFinal.getHolRot() to null if you want it to turn smoothly over
            // path. (Needs more testing)
            fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
                new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(),
                    fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
                Rotation2d.fromDegrees(180)
                    );
          }
        }
    
        // Declare an array to hold PathPoint objects made from all other points
        // specified in constructor.
        trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
        // Change trajectory based on alliance color
        return PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());
  }




}
