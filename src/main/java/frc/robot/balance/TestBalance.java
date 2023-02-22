// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.balance;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;




public class TestBalance extends CommandBase {
  /** Creates a new TestBalance. */





  private final DrivetrainSubsystem driveSystem;
  private final PoseEstimatorSubsystem poseEstimatorSystem;
  private final ProfiledPIDController xController = Constants.TeleopDriveConstants.xController;
  private final ProfiledPIDController yController = Constants.TeleopDriveConstants.yController;
  private final ProfiledPIDController omegaController = Constants.TeleopDriveConstants.omegaController;




  // This notates how many meters per second the robot should overshoot
  // the velocity for every meter away the robot is from the center of the
  // Charge Station. If you do want this to impact the velocity set this to zero.
  private static double velocityPerMeter = 1.0;

  private static double constantAddedVelocity = 0.0;

  // The range in meters from the center when the constantAddedVelocity will stop
  // being added to the total velocity.
  private static double posRangeConstantDisable = 0.0;

  // The range in radians from the 0 radians when the constantAddedVelocity
  // will stop being added to the total velocity.
  private static double pitchRangeConstantDisable = 0.0;

  // The range in meters from the center when all velocity will stop
  // being added.
  private static double posRangeDisable = 0.0;

  // The range in radians from the 0 radians when all velocity will stop
  // being added.
  private static double pitchRangeDisable = 0.0;

  // Limit how fast the robot can go at any time in meters per second
  private static double speedLimit = 1.5;



  // This code is only for testing! Remove for production!
  // --------------------------------------------------------------------------

  // Set to true if you want the force of gravity to impact the velocity of the
  // robot when going up the Charging Station.
  private boolean useGravity = true;

  // Set true you don't want constantAddedVelocity to be added within a certain
  // specified amount of meters from the center of the Charging Station.
  private boolean useDisableConstantOnPos = false;

  // Set true you don't want constantAddedVelocity to be added within a certain
  // specified amount of radians from 0 radians.
  private boolean useDisableConstantOnPitch = true;

  // Set true you don't want any velocity to be added within a certain
  // specified amount of meters from the center of the Charging Station.
  private boolean useDisableOnPos = false;

  // Set true you don't want any velocity to be added within a certain
  // specified amount of radians from 0 radians.
  private boolean useDisableOnPitch = true;

   // --------------------------------------------------------------------------


  double targetX = 3.94;

  // Only need blue side - Don't need side specifics.
  // double minTargetXRed = 4.03;
  // double maxTargetXRed = 6.47;


  private final double x, y;

    public TestBalance(DrivetrainSubsystem d, PoseEstimatorSubsystem p, double x, double y) {
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


  private double getDistanceFromX() {
    return poseEstimatorSystem.getCurrentPose().getX() - targetX;
  }

  private double getVelocity() {
    double pitch = driveSystem.getPitch();
    double distanceFromCenter = getDistanceFromX();
    double velocity = 0.0;
    if (
        (useDisableOnPitch && ((-pitchRangeDisable < pitch) && (pitchRangeDisable > pitch))) ||
        (useDisableOnPos && ((-posRangeDisable < distanceFromCenter) && (posRangeDisable > distanceFromCenter)))
      ) {

        velocity = 0.0;

      } else {
        // Calculating how much the robot accelerates downward based upon gravity and the angle of the platform.
        double velocityOfGravity = 9.81 * Math.sin(pitch) * (useGravity ? 1.0 : 0.0); //Ternary operator is for testing! DON'T use in production!
        double velocityOfDistance = velocityPerMeter * distanceFromCenter;
        double velocityAdded;
        // Checking if useDisableConstantOnPitch or useDisableConstantOnPos are in use and if so
        // we will set the velocity added to zero otherwise set it to our velocity constant.
        if (
          (useDisableConstantOnPitch && ((-pitchRangeConstantDisable < pitch) && (pitchRangeConstantDisable > pitch))) ||
          (useDisableConstantOnPos && ((-posRangeConstantDisable < distanceFromCenter) && (posRangeConstantDisable > distanceFromCenter)))
          ) {
            velocityAdded = 0;
          } else {
            velocityAdded = constantAddedVelocity;
          }
        velocity = velocityOfGravity + velocityOfDistance + velocityAdded;
      }

      if (velocity > speedLimit) {
        velocity = speedLimit;
      } else if (velocity < -speedLimit) {
        velocity = -speedLimit;
      }
      
      return velocity;
  }

  private void selfBalancing() {
    driveSystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(getVelocity(), 0.0, 0.0, poseEstimatorSystem.getCurrentPose().getRotation()));
  }
  
  @Override
  public void initialize() {
    driveSystem.resetPitch();
  }

  @Override
  public void execute() {
    selfBalancing();
  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
