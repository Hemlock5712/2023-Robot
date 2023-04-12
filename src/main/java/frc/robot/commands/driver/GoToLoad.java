// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.PPAStar;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class GoToLoad extends CommandBase {
  DrivetrainSubsystem drivetrain;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private final PathConstraints constraints;
  private final List<Obstacle> obstacles;
  private VisGraph AStarMap;
  private PPAStar pathfindCommand;

  /** Creates a new GoToPlace. */
  public GoToLoad(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator, PathConstraints constraints,
      List<Obstacle> obstacles, VisGraph AStarMap) {
    this.drivetrain = drivetrain;
    this.poseEstimatorSystem = poseEstimator;
    this.constraints = constraints;
    this.obstacles = obstacles;
    this.AStarMap = AStarMap;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // 7.08
    Node targetPosition = new Node(new Pose2d(14.6, 6.7, Rotation2d.fromDegrees(180)));
    pathfindCommand = new PPAStar(drivetrain, poseEstimatorSystem, constraints, targetPosition, obstacles, AStarMap,
        true);
    pathfindCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindCommand.isFinished();
  }
}
