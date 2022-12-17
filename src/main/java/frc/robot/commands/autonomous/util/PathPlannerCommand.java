package frc.robot.commands.autonomous.util;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Provides a base command structure for autonomous commands.
 * Requires all of the components, as well as setting up path
 * generation PID and generate methods.
 */
public class PathPlannerCommand extends SequentialCommandGroup {

  private ArrayList<PathPlannerTrajectory> trajectories;

  private DrivetrainSubsystem drivetrain;

  private String pathName;
  private PathConstraints constraint;
  private PathConstraints[] constraints;

  public static PIDController m_translationController = new PIDController(Constants.AutoConstants.X_kP, Constants.AutoConstants.X_kI, Constants.AutoConstants.X_kD);
  public static PIDController m_strafeController = new PIDController(Constants.AutoConstants.Y_kP, Constants.AutoConstants.Y_kI, Constants.AutoConstants.Y_kD);
  public static PIDController m_thetaController = new PIDController(Constants.AutoConstants.THETA_kP, Constants.AutoConstants.THETA_kI, Constants.AutoConstants.THETA_kD);

  private boolean pathHasBeenGenerated = false;

  public PathPlannerCommand(DrivetrainSubsystem drivetrain, String pathName, PathConstraints constraint, PathConstraints... constraints) {
    this.drivetrain = drivetrain;
    this.constraint = constraint;
    this.constraints = constraints;
    
    generate();
  }

  /**
   * Whether the paths have been generated
   * 
   * @return Has path been generated
   */
  public boolean hasBeenGenerated() {
    return pathHasBeenGenerated;
  }

  /**
   * Load and generate trajectories in this method. It is automatically called
   * upon creation of the command.
   */
  protected void generatePaths()
  {
    trajectories = PathPlanner.loadPathGroup(pathName, constraint, constraints);
  };

  /**
   * Generate the paths for the autonomous routine. If they have already
   * been generated, skip as to not cause delays.
   */
  public final void generate() {
    if (!pathHasBeenGenerated) {
      generatePaths();
      pathHasBeenGenerated = true;
    }
  }
}