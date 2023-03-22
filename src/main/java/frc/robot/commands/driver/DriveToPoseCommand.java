package frc.robot.commands.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {

  private static final double TRANSLATION_TOLERANCE = 0.02;

  private final ProfiledPIDController xController;

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final double goalPoseX;
  private final LEDSubsystem ledSubsystem;

  public DriveToPoseCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      double goalPoseX,
      LEDSubsystem ledSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.poseProvider = poseProvider;
    this.goalPoseX = goalPoseX;
    this.ledSubsystem = ledSubsystem;

    xController = Constants.TeleopDriveConstants.xController;
    xController.setTolerance(TRANSLATION_TOLERANCE);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
    xController.setGoal(goalPoseX);
  }

  public boolean atGoal() {
    return xController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    xController.reset(robotPose.getX());

  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    if (xSpeed == 0) {
      ledSubsystem.setLEDColor(0.77);
    }

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, 0, 0, robotPose.getRotation()));
  }

  @Override
  public boolean isFinished() {
    return atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

}