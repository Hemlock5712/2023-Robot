package frc.robot.commands.balance;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;

public class NewBalance extends CommandBase {
  private int state;
  private int debounceCount;
  private double robotSpeedSlow;
  private double robotSpeedFast;
  private double onChargeStationDegree;
  private double levelDegree;
  private double debounceTime;
  DrivetrainSubsystem drivetrain;
  private final PoseEstimatorSubsystem poseEstimatorSystem;

  public NewBalance(DrivetrainSubsystem d, PoseEstimatorSubsystem p) {
    this.drivetrain = d;
    this.poseEstimatorSystem = p;

    /**********
     * CONFIG *
     **********/
    // Speed the robot drived while scoring/approaching station, default = 0.4
    robotSpeedFast = 0.5;

    // Speed the robot drives while balancing itself on the charge station.
    // Should be roughly half the fast speed, to make the robot more accurate,
    // default = 0.2
    robotSpeedSlow = 0.3;

    // Angle where the robot knows it is on the charge station, default = 13.0
    onChargeStationDegree = 10.0;

    // Angle where the robot can assume it is level on the charging station
    // Used for exiting the drive forward sequence as well as for auto balancing,
    // default = 6.0
    levelDegree = 6.0;

    // Amount of time a sensor condition needs to be met before changing states in
    // seconds
    // Reduces the impact of sensor noice, but too high can make the auto run
    // slower, default = 0.2
    debounceTime = 0.2;

    addRequirements(drivetrain);

  }

  public double getPitch() {
    return drivetrain.getPitch();
  }

  public double getRoll() {
    return drivetrain.getRoll();
  }

  // returns the magnititude of the robot's tilt calculated by the root of
  // pitch^2 + roll^2, used to compensate for diagonally mounted rio
  public double getTilt() {
    double pitch = getPitch();
    double roll = getRoll();
    if ((pitch + roll) >= 0) {
      return Math.sqrt(pitch * pitch + roll * roll);
    } else {
      return -Math.sqrt(pitch * pitch + roll * roll);
    }
  }

  public int secondsToTicks(double time) {
    return (int) (time * 50);
  }

  // routine for automatically driving onto and engaging the charge station.
  // returns a value from -1.0 to 1.0, which left and right motors should be set
  // to.
  public double autoBalanceRoutine() {
    switch (state) {
      // drive forwards to approach station, exit when tilt is detected
      case 0:
        if (getTilt() > onChargeStationDegree) {
          debounceCount++;
        }
        if (getTilt() < -onChargeStationDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 1;
          debounceCount = 0;
          if (getTilt() < 0) {
            return -robotSpeedSlow;
          }
          return robotSpeedSlow;
        }
        if (getTilt() < 0) {
          return -robotSpeedFast;
        }
        return robotSpeedFast;
      // driving up charge station, drive slower, stopping when level
      case 1:
        if (getTilt() < levelDegree) {
          debounceCount++;
        }
        if (getTilt() > -levelDegree) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 2;
          debounceCount = 0;
          return 0;
        }
        return robotSpeedSlow;
      // on charge station, stop motors and wait for end of auto
      case 2:
        if (Math.abs(getTilt()) <= levelDegree / 2) {
          debounceCount++;
        }
        if (debounceCount > secondsToTicks(debounceTime)) {
          state = 4;
          debounceCount = 0;
          return 0;
        }
        if (getTilt() >= levelDegree) {
          return 0.1;
        } else if (getTilt() <= -levelDegree) {
          return -0.1;
        }
      case 3:
        return 0;
    }
    return 0;
  }

  private void selfBalancing() {
    System.out.println(getTilt());

    if (autoBalanceRoutine() == 0) {
      drivetrain.setWheelsToX();
    } else {
      drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(autoBalanceRoutine(), 0.0, 0.0,
          poseEstimatorSystem.getCurrentPose().getRotation()));
    }
  }

  @Override
  public void initialize() {
    state = 0;
    debounceCount = 0;
  }

  @Override
  public void execute() {
    selfBalancing();
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}
