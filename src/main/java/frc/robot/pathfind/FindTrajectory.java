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


  public static PathPlannerTrajectory getPathTrajectory(DrivetrainSubsystem driveSystem, PathConstraints constraints,
  List<Obstacle> obstacles, VisGraph AStarMap, Node startPoint, Node finalPosition){

    Node allianceFinal = finalPosition;
    // if(DriverStation.getAlliance() != Alliance.Blue){
    //   Pose2d flippedY = new Pose2d(startPoint.getX(),FieldConstants.fieldWidth-startPoint.getY(),startPoint.getHolRot());
    //   allianceFinal = new Node(finalPosition.getX(),FieldConstants.fieldWidth-finalPosition.getY(), finalPosition.getHolRot());
    //   startPoint = new Node(flippedY);
    // }

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
      return null;
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
            startPoint.getHolRot(), startingSpeed);
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
    return PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

  }

  public static PathPlannerTrajectory getPathTrajectoryAuto(DrivetrainSubsystem driveSystem, PathConstraints constraints,
  List<Obstacle> obstacles, VisGraph AStarMap, Node startPoint, Node finalPosition){

    Node allianceFinal = finalPosition;
    // if(DriverStation.getAlliance() != Alliance.Blue){
    //   Pose2d flippedY = new Pose2d(startPoint.getX(),FieldConstants.fieldWidth-startPoint.getY(),startPoint.getHolRot());
    //   allianceFinal = new Node(finalPosition.getX(),FieldConstants.fieldWidth-finalPosition.getY(), finalPosition.getHolRot());
    //   startPoint = new Node(flippedY);
    // }

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
      return null;
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
            startPoint.getHolRot(), startingSpeed);
      } else if (i + 1 == fullPath.size()) {
        fullPathPoints[i] = new PathPoint(new Translation2d(allianceFinal.getX(), allianceFinal.getY()),
            new Rotation2d(fullPath.get(i).getX() - fullPath.get(i - 1).getX(), fullPath.get(i).getY() - fullPath.get(i - 1).getY()),
            heading);
      } else {
        fullPathPoints[i] = new PathPoint(new Translation2d(fullPath.get(i).getX(), fullPath.get(i).getY()),
        new Rotation2d(fullPath.get(i + 1).getX() - fullPath.get(i).getX(), fullPath.get(i + 1).getY() - fullPath.get(i).getY()),
        heading);
      }
    }

    // Declare an array to hold PathPoint objects made from all other points
    // specified in constructor.
    trajectory = PathPlanner.generatePath(constraints, Arrays.asList(fullPathPoints));
    //var alliance = Alliance.Blue;
    //trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, alliance);
    return PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, DriverStation.getAlliance());

  }

}
