package frc.robot.commands.driver;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
        TargetLevel level = Position.getPlacementPosition().getLevel();

        Command goToEndArmPosition = null;
        Command goToEndArmPositionTransit = null;

        switch(level) {
            case Top:
                goToEndArmPosition = new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.HIGH_PEG);
                goToEndArmPositionTransit = new MoveArmToSetpointNoExtend(elevator, extension, wrist, Constants.ArmSetpoints.HIGH_PEG);
                break;
            case Mid:
                goToEndArmPosition = new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.MID_PEG);
                goToEndArmPositionTransit = new MoveArmToSetpointNoExtend(elevator, extension, wrist, Constants.ArmSetpoints.MID_PEG);
                break;
            case Low:
                goToEndArmPosition = new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.HYBRID_NODE);
                goToEndArmPositionTransit = new MoveArmToSetpointNoExtend(elevator, extension, wrist, Constants.ArmSetpoints.HYBRID_NODE);
                break;
        }

        addCommands(new ParallelCommandGroup(
                goToEndArmPositionTransit,
                new GoToPlace(drivetrain, poseEstimator, constraints, obstacles, AStarMap)),
                goToEndArmPosition
        );
    }
}
