package frc.robot.commands.driver;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.operator.MoveArmToSetpoint;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class GoToLoadWithArm extends SequentialCommandGroup {
  public GoToLoadWithArm(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator,
      PathConstraints constraints,
      List<Obstacle> obstacles, VisGraph AStarMap, ExtensionSubsystem extension, ElevatorSubsystem elevator,
      WristSubsystem wrist, IntakeSubsystem intake) {
    addCommands(new ParallelDeadlineGroup(
          new GoToLoad(drivetrain, poseEstimator, constraints, obstacles, AStarMap),
          new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.TRANSIT)), 
        new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.SINGLE_SUBSTATION_PICKUP).alongWith(new RunIntakeCommand(intake)));
  }
}
