// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balance;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class Balance extends CommandBase {

  double speed = Units.inchesToMeters(15);
  double positionThreshold = 3;
  double velocityThreshold = 8;

  DrivetrainSubsystem drivetrain;
  PoseEstimatorSubsystem poseEstimator;
  private double angleDegrees;

  /** Creates a new Balance. */
  public Balance(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.drivetrain = drivetrain;
    this.poseEstimator = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleDegrees = Double.POSITIVE_INFINITY;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d rotation = poseEstimator.getCurrentPose().getRotation();

    SmartDashboard.putNumber("Roll", Units.radiansToDegrees(drivetrain.getRoll()));
    SmartDashboard.putNumber("Pitch", Units.radiansToDegrees(drivetrain.getPitch()));

    angleDegrees = rotation.getCos() * drivetrain.getRoll()
                    + rotation.getSin() * drivetrain.getPitch();

    SmartDashboard.putNumber("AngleDegrees", angleDegrees);

    double angleVelocityDegreesPerSec =
            rotation.getCos() * Units.radiansToDegrees(drivetrain.getRollDPS())
                    + rotation.getSin() * Units.radiansToDegrees(drivetrain.getPitchDPS());
    boolean shouldStop =
            (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThreshold)
                    || (angleDegrees > 0.0
                    && angleVelocityDegreesPerSec < -velocityThreshold);

    if (shouldStop) {
      drivetrain.stop();
    } else {
      drivetrain.drive(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                      Units.inchesToMeters(speed) * (angleDegrees > 0.0 ? -1.0 : 1.0),
                      0.0,
                      0.0,
                      rotation));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setWheelsToX();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angleDegrees) < positionThreshold;
  }
}
