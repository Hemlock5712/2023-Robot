package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class DriveToPoint extends CommandBase {


  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private final ProfiledPIDController xController = Constants.TeleopDriveConstants.xController;
  private final ProfiledPIDController yController = Constants.TeleopDriveConstants.yController;
  private final ProfiledPIDController omegaController = Constants.TeleopDriveConstants.omegaController;


  private final double x, y;

  public DriveToPoint(DrivetrainSubsystem d, PoseEstimatorSubsystem p, double x, double y) {
    this.driveSystem = d;
    poseEstimatorSystem = p;

    this.x = x;
    this.y = y;

    this.xController.setTolerance(0.2);
    
    this.yController.setTolerance(0.2);

    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(driveSystem, poseEstimatorSystem);
  }

  @Override
  public void initialize() {
    var robotPose = poseEstimatorSystem.getCurrentPose();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
  }

  @Override
  public void execute() {
        // Drive
        xController.setGoal(x);
        yController.setGoal(y);
        omegaController.setGoal(Math.toRadians(180.0));
    
      var robotPose = poseEstimatorSystem.getCurrentPose();

      // Drive to the target
      var xSpeed =  xController.calculate(robotPose.getX());
      if ( xController.atGoal()) {
        xSpeed = 0;
      }

      var ySpeed =  yController.calculate(robotPose.getY());
      if ( yController.atGoal()) {
        ySpeed = 0;
      }

      var omegaSpeed =  omegaController.calculate(robotPose.getRotation().getRadians());
      if ( omegaController.atGoal()) {
        omegaSpeed = 0;
      }

      driveSystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    }

  @Override
  public void end(boolean interrupted) {
    driveSystem.stop();
  }

  @Override
  public boolean isFinished()
  {
    return false;
  }
}
