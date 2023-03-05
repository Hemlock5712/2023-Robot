package frc.robot.commands.driver;

import java.util.List;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.operator.MoveArmToSetpoint;
import frc.robot.commands.operator.Position;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.PlacementPosition;
import frc.robot.util.TargetLevel;

public class GoToPlaceWithArm extends SequentialCommandGroup {

  public GoToPlaceWithArm(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator,
      PathConstraints constraints,
      List<Obstacle> obstacles, VisGraph AStarMap, ExtensionSubsystem extension, ElevatorSubsystem elevator,
      WristSubsystem wrist) {

        PlacementPosition temp = Position.getPlacementPosition();
        TargetLevel level = temp.getLevel();
        ArmSetpoint setpoint = Constants.ArmSetpoints.HYBRID_NODE;

        switch(level) {
          case Top:
            setpoint = Constants.ArmSetpoints.HIGH_PEG;
            break;
          case Mid:
            setpoint = Constants.ArmSetpoints.MID_PEG;
            break;
          case Low:
            setpoint = Constants.ArmSetpoints.HYBRID_NODE;
            break;
        }
        double tempAngle = setpoint.getAngle();
        double tempWrist = setpoint.getWristAngle();
        System.out.println("---------------------------SETPOINT:"+setpoint.getLength());

        addCommands(new ParallelDeadlineGroup(
            new GoToPlace(drivetrain, poseEstimator, constraints, obstacles, AStarMap),
            new MoveArmToSetpoint(elevator, extension, wrist, new ArmSetpoint(tempAngle, 0, tempWrist))),
            new MoveArmToSetpoint(elevator, extension, wrist, setpoint));
  }
}
