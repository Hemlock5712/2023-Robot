package frc.robot.commands.driver;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command to drive to a pose.
 */
public class DriveToPoseCommand extends CommandBase {

  private static final double TRANSLATION_TOLERANCE = 0.02;
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController thetaController;

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Pose2d> poseProvider;
  private final double goalPoseX;
  private final LEDSubsystem ledSubsystem;

  public DriveToPoseCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Pose2d> poseProvider,
      double goalPoseX,
      LEDSubsystem ledSubsystem,
      boolean useAllianceColor) {
    this(drivetrainSubsystem, poseProvider, goalPoseX, ledSubsystem);
  }

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
    yController = Constants.TeleopDriveConstants.yController;
    xController.setTolerance(TRANSLATION_TOLERANCE);
    yController.setTolerance(TRANSLATION_TOLERANCE);
    thetaController = Constants.TeleopDriveConstants.omegaController;
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    resetPIDControllers();
    thetaController.setGoal(poseProvider.get().getRotation().getRadians());
    xController.setGoal(goalPoseX);
    yController.setGoal(poseProvider.get().getY());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }

  private void resetPIDControllers() {
    var robotPose = poseProvider.get();
    thetaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
    var robotPose = poseProvider.get();
    // Drive to the goal
    var xSpeed = xController.calculate(robotPose.getX());
    if (xController.atGoal()) {
      xSpeed = 0;
    }

    var ySpeed = yController.calculate(robotPose.getY());
    if (yController.atGoal()) {
      ySpeed = 0;
    }

    var omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
    if (thetaController.atGoal()) {
      omegaSpeed = 0;
    }

    if (xSpeed == 0 || ySpeed == 0 || omegaSpeed == 0) {
      // TODO JOSH
    }

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
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