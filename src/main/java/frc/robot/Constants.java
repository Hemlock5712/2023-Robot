// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.XYACalulator;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DrivetrainConstants {

    public static Alliance alliance = Alliance.Invalid;

    public static final boolean ADD_TO_DASHBOARD = true;

    /**
     * The left-to-right distance between the drivetrain wheels
     * <p>
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.57785;
    /**
     * The front-to-back distance between the drivetrain wheels.
     * <p>
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.57785;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(145.19 + 180);
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(-325.722-295.837+231.41-3);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(297.42 + 180);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(-120.84-8.66+248.112-1.8);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(2.54 + 180);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(177.891+3.9);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(310.07 + 180);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(-131.748-90-14.11+3.7);

    public static final int PIGEON_ID = 13;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        ModuleConfiguration.MK4I_L2.getDriveReduction() *
        ModuleConfiguration.MK4I_L2.getWheelDiameter() * PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = (DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND
        /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

    /**
     * Voltage needed to overcome the motor’s static friction. kS
     */
    public static final double DRIVE_kS = 0.6716;
    /**
     * Voltage needed to hold (or "cruise") at a given constant velocity. kV
     */
    public static final double DRIVE_kV = 2.3;
    /**
     * Voltage needed to induce a given acceleration in the motor shaft. kA
     */
    public static final double DRIVE_kA = 0.52878;

    public static final double STEER_kP = 0.2;
    public static final double STEER_kI = 0.001;
    public static final double STEER_kD = 0.0;

    public static final double DRIVE_kP = 0.04;
    public static final double DRIVE_kI = 0.0;
    public static final double DRIVE_kD = 0.0;

  }

  public static final class TeleopDriveConstants {

    public static final double DEADBAND = 0.1;

    public static final double X_RATE_LIMIT = 7.75;
    public static final double Y_RATE_LIMIT = 7.75;
    public static final double ROTATION_RATE_LIMIT = 5.0 * PI;

    public static final double HEADING_MAX_VELOCITY = PI * 4;
    public static final double HEADING_MAX_ACCELERATION = PI * 16;

    public static final double HEADING_kP = 2.0;
    public static final double HEADING_kI = 0.0;
    public static final double HEADING_kD = 0.0;

    public static final double HEADING_TOLERANCE = degreesToRadians(1.5);

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);

    public static final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    public static final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    public static final ProfiledPIDController omegaController = new ProfiledPIDController(2, 0, 0, OMEGA_CONSTRAINTS);

  }

  public static class VisionConstants {

    public static boolean USE_VISION = true;

    /**
     * Physical location of the right camera on the robot, relative to the center of
     * the
     * robot.
     */
    public static final Transform3d ROBOT_TO_RIGHT_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(-6.88), Units.inchesToMeters(31.09)),
        new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45)));

    /**
     * Physical location of the left camera on the robot, relative to the center of
     * the
     * robot.
     */
    public static final Transform3d ROBOT_TO_LEFT_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-11.88), Units.inchesToMeters(6.88), Units.inchesToMeters(31.09)),
        new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(46)));

    /**
     * Physical location of the back camera on the robot, relative to the center of
     * the
     * robot.
     */
    public static final Transform3d ROBOT_TO_BACK_CAMERA = new Transform3d(
        new Translation3d(Units.inchesToMeters(-13.76), Units.inchesToMeters(6.1), Units.inchesToMeters(33.65)),
        new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(180)));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            1, // x
            1, // y
            1 * Math.PI // theta
        );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = Matrix.mat(Nat.N3(), Nat.N1())
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            .1, // x
            .1, // y
            .1);

  }

  public static class AutoConstants {
    public static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static double THETA_kP = 2.75;
    public static double THETA_kI = 0.0;
    public static double THETA_kD = 0.0;

    public static double X_kP = 5;
    public static double X_kI = 0.0;
    public static double X_kD = 0.0;

    public static double Y_kP = 5;
    public static double Y_kI = 0.0;
    public static double Y_kD = 0.0;

    public static PIDConstants translationConstants = new PIDConstants(X_kP, X_kI, X_kD);
    public static PIDConstants rotationConstants = new PIDConstants(THETA_kP, THETA_kI, THETA_kD);

    public static PIDController translationController = new PIDController(X_kP, X_kI, X_kD);
    public static PIDController strafeController = new PIDController(Y_kP, Y_kI, Y_kD);
    public static PIDController thetaController = new PIDController(THETA_kP, THETA_kI, THETA_kD);

  }

  public static class PneumaticsConstants {
    public static final double MIN_PRESSURE = 90;
    public static final double MAX_PRESSURE = 120;
  }

  public static class ArmConstants {
    /**
     * CAN ID of the arm angle encoder (at the pivot point)
     */
    public static final int ARM_ANGLE_ENCODER_ID = 20;
    /**
     * CAN ID of the front elevator motor
     */
    public static final int ELEVATOR_FRONT_FALCON_ID = 26;
    /**
     * CAN ID of the back elevator motor
     */
    public static final int ELEVATOR_BACK_FALCON_ID = 23;
    /**
     * Radius of the spool in meters
     */
    public static final double DRUM_RADIUS = Units.inchesToMeters(5.0 / 8.0);
    /**
     * Mass of the arm in kg
     */
    public static final double ARM_MASS = Units.lbsToKilograms(65);
    /**
     * Maximum speed of the arm in m/s
     * <p>
     * This should be tuned later once we determine how fast we want it to move
     */
    public static final double MAX_SPEED = 0.05;
    /**
     * Maximum acceleration of the arm in m/s^2
     * <p>
     * This should be tuned later once we determine how fast we want it to move
     */
    public static final double MAX_ACCELERATION = 0.05;
    /**
     * Gear ratio of the elevator from the motors to the spool
     */
    public static final double ELEVATOR_GEARING = 3.80;
    /**
     * Height of the pivot point from the ground.
     */
    public static final double PIVOT_POINT_HEIGHT = Units.inchesToMeters(21);
    /**
     * How far out the mount point is from the pivot point.
     */
    public static final double MOUNT_POINT_DISTANCE_ON_ARM = Units.inchesToMeters(21);
    /**
     * Angle offset of CANCoder relative to the arm. This is the angle between the
     * arm and the encoder's zero position.
     * <p>
     * Should be set to 0 when the arm is parallel to the ground.
     */
    public static final double ARM_ANGLE_ABSOLUTE_OFFSET = -155.4785;
    public static final double MAX_ARM_LENGTH = Units.inchesToMeters(40);
    public static final double MIN_ARM_LENGTH = Units.inchesToMeters(10);
    /**
     * Tolerance for when the elevator considers itself to be at the target point
     */
    public static final double AT_TARGET_TOLERANCE = Units.inchesToMeters(2);
  }

  public static class ExtensionConstants {
    /**
     * CAN ID of the extension encoder
     */
    public static final int EXTENSION_FALCON_ID = 24;
    /**
     * Gear ratio of the extension from the motor to the spool
     */
    public static final double EXTENSION_GEARING = 1;
    /**
     * Mass of the extension in kg
     */
    public static final double EXTENSION_MASS = Units.lbsToKilograms(66);
    /**
     * Maximum speed of the extension in m/s
     * <p>
     * This should be tuned later once we determine how fast we want it to move
     */
    public static final double MAX_SPEED = 0.2;
    /**
     * Maximum acceleration of the extension in m/s^2
     * <p>
     * This should be tuned later once we determine how fast we want it to move
     */
    public static final double MAX_ACCELERATION = 0.2;
    /**
     * Maximum length of the extension in meters
     */
    public static final double EXTENSION_LENGTH = Units.inchesToMeters(48);
    /**
     * Radius of the spool in meters
     */
    public static final double SPOOL_RADIUS = Units.inchesToMeters(1);

    /**
     * Circumference of the spool in meters
     */
    public static final double SPOOL_CIRCUMFERENCE = 2 * Math.PI * SPOOL_RADIUS;
    /**
     * Tolerance for when the extension considers itself to be at the target point
     */
    public static final double AT_TARGET_TOLERANCE = Units.inchesToMeters(2);
    public static final double MAX_VOLTAGE = 5.0;
    public static final double MAX_EXTENSION = Units.inchesToMeters(80);
    public static final double MIN_EXTENSION = .2;
  }

  public static class WristConstants {
    public static final int MOTOR_ID = 25;
    public static final int TICKS_PER_REVOLUTION = 42;
    public static final double GEAR_RATIO = 49;
    public static final int ENCODER_ID = 40;
  }

  public static class IntakeConstants {
    public static final int MOTOR_ID = 33;
    public static final int TICKS_PER_REVOLUTION = 4096;
    public static final double GEAR_RATIO = 49;
  }

  public static class SpacerConstants {
    public static final int MOTOR_ID = 29;
  }

  // These are all very arbitrary and need to be tuned
  public static class ArmSetpoints {
    /**
     * Put arm on ground to pick up a cone that's lying down
     */
    public static final ArmSetpoint GROUND_CUBE_PICKUP = new ArmSetpoint(-11, 0, -40);
    /**
     * Put arm above ground to pick up a cone that's standing up
     */
    public static final ArmSetpoint ABOVE_CONE_PICKUP = new ArmSetpoint(0, 0, -45);
    /**
     * Put arm down onto the cone that's standing up to pick it up
     */
    public static final ArmSetpoint ABOVE_CONE_PICKUP_2 = new ArmSetpoint(-15, Units.inchesToMeters(10), -25);
    /**
     * Put arm into a safe position for transit across the field
     */
    // public static final ArmSetpoint TRANSIT = XYACalulator.Calulator(0, 0, 90);
    public static final ArmSetpoint TRANSIT = new ArmSetpoint(0, -0.06, 90);
    /**
     * Extend the arm out to place cone on the high peg
     */
    public static final ArmSetpoint HIGH_PEG = new ArmSetpoint(35, Units.inchesToMeters(30), -50);
    /**
     * Extend the arm out to place cone on the mid peg
     */
    // public static final ArmSetpoint MID_PEG = XYACalulator.Calulator(-13, 11,
    // -20);

    public static final ArmSetpoint MID_PEG = new ArmSetpoint(31, Units.inchesToMeters(10.5), -48);

    public static final ArmSetpoint AUTO_MID_PEG = XYACalulator.Calulator(-1, 17, -20);

    /**
     * Extend the arm out to place cone on the high peg
     */
    public static final ArmSetpoint HIGH_CUBE = XYACalulator.Calulator(18, 28, -20);
    /**
     * Extend the arm out to place cone on the mid peg
     */
    public static final ArmSetpoint MID_CUBE = XYACalulator.Calulator(-1, 14, -20);
    /**
     * Extend the arm out to place on the hybrid node
     */
    public static final ArmSetpoint HYBRID_NODE = XYACalulator.Calulator(0, 0, -45);
    /**
     * Put arm out upside down to pick up a cone direct from the single substation
     */
    // public static final ArmSetpoint SINGLE_SUBSTATION_PICKUP =
    // XYACalulator.Calulator(-.2, 2, 34);
    // WAS 34 AT MIDLAND
    public static final ArmSetpoint SINGLE_SUBSTATION_PICKUP = new ArmSetpoint(0, -.06, 45);
    // public static final ArmSetpoint TEST_SINGLE_SUBSTATION_PICKUP = new
    // ArmSetpoint(0, -.06, 45);
    public static final ArmSetpoint STARTING_CONFIG = new ArmSetpoint(71, 0, -106);

    public static final ArmSetpoint GROUND_CONE_PICKUP = new ArmSetpoint(-4, 0, -10);
  }

}
