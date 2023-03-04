package frc.robot.commands.driver;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.operator.MoveArmToPlaceItem;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GoToPlaceWithArm extends SequentialCommandGroup {
  public GoToPlaceWithArm(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator,
      PathConstraints constraints,
      List<Obstacle> obstacles, VisGraph AStarMap, ExtensionSubsystem extension, ElevatorSubsystem elevator,
      WristSubsystem wrist) {
    addCommands(new ParallelDeadlineGroup(
        new GoToPlace(drivetrain, poseEstimator, constraints, obstacles, AStarMap),
        new MoveArmToPlaceItem(elevator, extension, wrist, false)),
        new MoveArmToPlaceItem(elevator, extension, wrist, true));
  }
}
