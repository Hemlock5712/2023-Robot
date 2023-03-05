// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.driver;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.PPAStar;
import frc.robot.commands.operator.Position;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.FieldConstants;
import frc.robot.util.TargetPosition;

public class GoToPlace extends CommandBase {
  DrivetrainSubsystem drivetrain;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private final PathConstraints constraints;
  private final List<Obstacle> obstacles;
  private VisGraph AStarMap;
  private PPAStar pathfindCommand;

  /** Creates a new GoToPlace. */
  public GoToPlace(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator, PathConstraints constraints,
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
    TargetPosition target = Position.getPlacementPosition().getPosition();
    Pose2d posePosition = FieldConstants.PlacementPositions.get(target);
    Node targetPosition = new Node(posePosition);
    pathfindCommand = new PPAStar(drivetrain, poseEstimatorSystem, constraints, targetPosition, obstacles, AStarMap, false);
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
