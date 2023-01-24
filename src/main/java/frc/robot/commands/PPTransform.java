package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;

public class PPTransform {

  /**
   * Create a new path with the transformed paths
   *
   * @param pathGroup Path group to follow during the auto
   * @return flippedPath The flipped trajectories
   */
  public static List<PathPlannerTrajectory> flipTraj(List<PathPlannerTrajectory> pathGroup) {
    List<PathPlannerTrajectory> flippedPath = new ArrayList<>();
    for (PathPlannerTrajectory traj : pathGroup) {
      flippedPath.add(PathPlannerTrajectory.transformTrajectoryForAlliance(
            traj,
            DriverStation.getAlliance()
        ));
    }
    return flippedPath;
  }
}
