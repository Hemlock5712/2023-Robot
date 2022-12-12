package frc.robot.commands.autonomous.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Provides a base command structure for autonomous commands.
 * Requires all of the components, as well as setting up path
 * generation PID and generate methods.
 */
public abstract class AutoBaseCommand extends SequentialCommandGroup {

  protected DrivetrainSubsystem drivetrain;

  protected PIDController m_translationController = new PIDController(Constants.AutoConstants.X_kP, Constants.AutoConstants.X_kI, Constants.AutoConstants.X_kD);
  protected PIDController m_strafeController = new PIDController(Constants.AutoConstants.Y_kP, Constants.AutoConstants.Y_kI, Constants.AutoConstants.Y_kD);
  protected PIDController m_thetaController = new PIDController(Constants.AutoConstants.THETA_kP, Constants.AutoConstants.THETA_kI, Constants.AutoConstants.THETA_kD);

  private boolean pathHasBeenGenerated = false;

  public AutoBaseCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
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
  protected abstract void generatePaths();

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