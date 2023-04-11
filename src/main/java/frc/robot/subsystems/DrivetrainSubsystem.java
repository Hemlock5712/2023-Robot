// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.BACK_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_LEFT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_ENCODER;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_MOTOR;
import static frc.robot.Constants.DrivetrainConstants.FRONT_RIGHT_MODULE_STEER_OFFSET;
import static frc.robot.Constants.DrivetrainConstants.PIGEON_ID;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.IntStream;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.swerve.ProSwerveModule;
import frc.robot.swerve.SwerveModule;
import frc.robot.swerve.SwerveModuleConstants;
import frc.robot.swerve.SwerveSpeedController;
import frc.robot.swerve.SwerveSteerController;

public class DrivetrainSubsystem extends SubsystemBase {

  private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(PIGEON_ID);
  private final ProSwerveModule[] swerveModules;

  private static final NetworkTable moduleStatesTable = NetworkTableInstance.getDefault().getTable("SwerveStates");
  NetworkTableEntry angleEntry = NetworkTableInstance.getDefault().getTable("DrivetrainSubsystem").getEntry("angle");

  private ChassisSpeeds desiredChassisSpeeds;

  public DrivetrainSubsystem() {

    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
    pigeon.configMountPoseRoll(0);
    pigeon.configMountPoseYaw(0);
    pigeon.configMountPosePitch(0);

    ShuffleboardLayout frontLeftLayout = null;
    ShuffleboardLayout frontRightLayout = null;
    ShuffleboardLayout backLeftLayout = null;
    ShuffleboardLayout backRightLayout = null;

    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      frontLeftLayout = tab.getLayout("Front Left Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(0, 0);

      frontRightLayout = tab.getLayout("Front Right Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(2, 0);

      backLeftLayout = tab.getLayout("Back Left Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(4, 0);

      backRightLayout = tab.getLayout("Back Right Module", BuiltInLayouts.kGrid)
          .withProperties(Map.of("Number of columns", 1, "Number of rows", 6))
          .withSize(2, 4)
          .withPosition(6, 0);
    }

    swerveModules = new ProSwerveModule[] {
        new ProSwerveModule(
          new SwerveModuleConstants()
          .fromModuleConfiguration(ModuleConfiguration.MK4I_L2)
            .withDriveMotorId(FRONT_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotorId(FRONT_LEFT_MODULE_STEER_MOTOR)
            .withCANcoderId(FRONT_LEFT_MODULE_STEER_ENCODER)
            .withCANcoderOffset(FRONT_LEFT_MODULE_STEER_OFFSET)),
        new ProSwerveModule(
          new SwerveModuleConstants()
          .fromModuleConfiguration(ModuleConfiguration.MK4I_L2)
            .withDriveMotorId(FRONT_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotorId(FRONT_RIGHT_MODULE_STEER_MOTOR)
            .withCANcoderId(FRONT_RIGHT_MODULE_STEER_ENCODER)
            .withCANcoderOffset(FRONT_RIGHT_MODULE_STEER_OFFSET)),
        new ProSwerveModule(
          new SwerveModuleConstants()
          .fromModuleConfiguration(ModuleConfiguration.MK4I_L2)
            .withDriveMotorId(BACK_LEFT_MODULE_DRIVE_MOTOR)
            .withSteerMotorId(BACK_LEFT_MODULE_STEER_MOTOR)
            .withCANcoderId(BACK_LEFT_MODULE_STEER_ENCODER)
            .withCANcoderOffset(BACK_LEFT_MODULE_STEER_OFFSET)),
        new ProSwerveModule(
          new SwerveModuleConstants()
          .fromModuleConfiguration(ModuleConfiguration.MK4I_L2)
            .withDriveMotorId(BACK_RIGHT_MODULE_DRIVE_MOTOR)
            .withSteerMotorId(BACK_RIGHT_MODULE_STEER_MOTOR)
            .withCANcoderId(BACK_RIGHT_MODULE_STEER_ENCODER)
            .withCANcoderOffset(BACK_RIGHT_MODULE_STEER_OFFSET) )};

    // Put all the modules into brake mode
    for (ProSwerveModule swerveModule : swerveModules) {
      swerveModule.setNeutralMode(NeutralModeValue.Brake);
    }

    pigeon.zeroGyroBiasNow();
  }

  /**
   * Creates a server module instance
   * 
   * @param container           shuffleboard layout, or null
   * @param moduleConfiguration module configuration
   * @param driveMotorPort      drive motor CAN ID
   * @param steerMotorPort      steer motor CAN ID
   * @param steerEncoderPort    steer encoder CAN ID
   * @param steerOffset         offset for steer encoder
   * @return new swerve module instance
   */
  private static SwerveModule createSwerveModule(
      ShuffleboardLayout container,
      ModuleConfiguration moduleConfiguration,
      int driveMotorPort,
      int steerMotorPort,
      int steerEncoderPort,
      double steerOffset) {

    return new SwerveModule(
        new SwerveSpeedController(driveMotorPort, moduleConfiguration, container),
        new SwerveSteerController(steerMotorPort, steerEncoderPort, steerOffset, container, moduleConfiguration));
  }

  public Rotation2d getGyroscopeRotation() {
    return pigeon.getRotation2d();
  }

  /**
   * Gets the raw gyro data.
   * 
   * @return x[0], y[1], and z[2] data in degrees per second
   */
  public double[] getGyroVelocityXYZ() {
    double[] xyz = new double[3];
    pigeon.getRawGyro(xyz);
    return xyz;
  }

  /**
   * Sets the desired chassis speeds
   * 
   * @param chassisSpeeds desired chassis speeds
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    desiredChassisSpeeds = chassisSpeeds;
  }

  /**
   * Set the wheels to an X pattern to plant the robot.
   */
  public void setWheelsToX() {
    desiredChassisSpeeds = null;
    setModuleStates(new SwerveModuleState[] {
        // front left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
        // front right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
        // back left
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(135.0)),
        // back right
        new SwerveModuleState(0.0, Rotation2d.fromDegrees(-135.0))
    });
  }

  /**
   * Sets the desired speeds to zero
   */
  public void stop() {
    drive(new ChassisSpeeds());
  }

  /**
   * Gets the actual chassis speeds
   * 
   * @return actual chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  @Override
  public void periodic() {
    // Set the swerve module states
    if (desiredChassisSpeeds != null) {
      var desiredStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(desiredChassisSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND);
      setModuleStates(desiredStates);
    }
    // Module states for Advantage Scope
    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      double[] moduleStateArray = new double[swerveModules.length * 2];
      for (int i = 0; i < swerveModules.length; i++) {
        var module = swerveModules[i];
        moduleStateArray[i * 2] = module.getSteerAngle().getDegrees();
        moduleStateArray[(i * 2) + 1] = module.getDriveVelocity();
      }
      moduleStatesTable.getEntry("Measured").setDoubleArray(moduleStateArray);
      moduleStatesTable.getEntry("Rotation").setDouble(getGyroscopeRotation().getDegrees());
    }
    // Always reset desiredChassisSpeeds to null to prevent latching to the last
    // state (aka motor safety)!!
    desiredChassisSpeeds = null;
    angleEntry.setNumber(getRoll());
  }

  /**
   * Gets the current drivetrain state (velocity, and angle), as reported by the
   * modules themselves.
   * 
   * @return current drivetrain state. Array orders are frontLeft, frontRight,
   *         backLeft, backRight
   */
  private SwerveModuleState[] getModuleStates() {
    return Arrays.stream(swerveModules).map(module -> module.getState()).toArray(SwerveModuleState[]::new);
  }

  /**
   * Gets the current drivetrain position, as reported by the modules themselves.
   * 
   * @return current drivetrain state. Array orders are frontLeft, frontRight,
   *         backLeft, backRight
   */
  public SwerveModulePosition[] getModulePositions() {
    return Arrays.stream(swerveModules).map(module -> module.getPosition()).toArray(SwerveModulePosition[]::new);
  }

  /**
   * Sets the states of the modules.
   * 
   * @param states array of states. Must be ordered frontLeft, frontRight,
   *               backLeft, backRight
   */
  public void setModuleStates(SwerveModuleState[] states) {
    double[] moduleSetpointArray = new double[states.length * 2];
    IntStream.range(0, swerveModules.length).forEach(i -> {
      var swerveModule = swerveModules[i];
      var desiredState = SwerveModuleState.optimize(states[i], swerveModule.getSteerAngle());
      swerveModule.apply(desiredState);

      // Module setpoints for Advantage Scope
      if (DrivetrainConstants.ADD_TO_DASHBOARD) {
        moduleSetpointArray[i * 2] = desiredState.angle.getDegrees();
        moduleSetpointArray[(i * 2) + 1] = desiredState.speedMetersPerSecond;
      }
    });
    if (DrivetrainConstants.ADD_TO_DASHBOARD) {
      moduleStatesTable.getEntry("Setpoints").setDoubleArray(moduleSetpointArray);
    }
  }

  /**
   * Reseeds the Talon FX steer motors from their CANCoder absolute position.
   * Workaround for "dead wheel"
   */
  public void reseedSteerMotorOffsets() {
    // Arrays.stream(swerveModules).forEach(SwerveModule::reseedSteerMotorOffset);
  }

  /**
   * Creates a command to follow a Trajectory on the drivetrain.
   * 
   * @param trajectory trajectory to follow
   * @return command that will run the trajectory
   */
  public Command createCommandForTrajectory(Trajectory trajectory, Supplier<Pose2d> poseSupplier) {
    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_kP, AutoConstants.THETA_kI, AutoConstants.THETA_kD, AutoConstants.THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        poseSupplier,
        DrivetrainConstants.KINEMATICS,
        Constants.AutoConstants.translationController,
        Constants.AutoConstants.strafeController,
        thetaController,
        this::setModuleStates);

    return swerveControllerCommand;
  }

  public static PPSwerveControllerCommand followTrajectory(DrivetrainSubsystem d, PoseEstimatorSubsystem s,
      PathPlannerTrajectory traj) {
    Constants.AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);
    return new PPSwerveControllerCommand(
        traj,
        s::getCurrentPose,
        Constants.DrivetrainConstants.KINEMATICS,
        Constants.AutoConstants.translationController,
        Constants.AutoConstants.strafeController,
        Constants.AutoConstants.thetaController,
        d::setModuleStates,
        false);
  }

  public double getRoll() {
    return pigeon.getRoll() + 2;
  }

  public double getPitch() {
    return pigeon.getPitch();
  }

  public double getPitchDPS() {
    double[] xyz = getGyroVelocityXYZ();
    return Units.degreesToRadians(-xyz[0]);
  }

  public double getRollDPS() {
    double[] xyz = getGyroVelocityXYZ();
    return Units.degreesToRadians(xyz[1]);
  }

}