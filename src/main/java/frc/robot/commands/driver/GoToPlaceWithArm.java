package frc.robot.commands.driver;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.operator.MoveArmToPlaceItem;
import frc.robot.commands.operator.MoveArmToSetpoint;
import frc.robot.commands.operator.MoveArmToSetpointNoExtend;
import frc.robot.commands.operator.Position;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.*;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.TargetLevel;

import java.util.List;

public class GoToPlaceWithArm extends SequentialCommandGroup {
    public GoToPlaceWithArm(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator, PathConstraints constraints,
                            List<Obstacle> obstacles, VisGraph AStarMap, ExtensionSubsystem extension, ElevatorSubsystem elevator, WristSubsystem wrist) {
        addCommands(new ParallelCommandGroup(
                        new MoveArmToPlaceItem(elevator, extension, wrist, false),
                        new GoToPlace(drivetrain, poseEstimator, constraints, obstacles, AStarMap)),
                    new MoveArmToPlaceItem(elevator, extension, wrist, true)
        );
    }
}
