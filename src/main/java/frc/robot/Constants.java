// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.swerve.ModuleConfiguration;
import frc.robot.util.ArmSetpoint;

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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(145.19 + 180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(297.42 + 180);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(2.54 + 180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(11.65 + 180);

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
        public static final double DRIVE_kV = 2.5913;
        /**
         * Voltage needed to induce a given acceleration in the motor shaft. kA
         */
        public static final double DRIVE_kA = 0.19321;

        public static final double STEER_kP = 0.2;
        public static final double STEER_kI = 0.0;
        public static final double STEER_kD = 0.1;

        public static final double DRIVE_kP = 0.02;
        public static final double DRIVE_kI = 0.0;
        public static final double DRIVE_kD = 0.0;

    }

    public static final class TeleopDriveConstants {

        public static final double DEADBAND = 0.1;

        public static final double X_RATE_LIMIT = 6.0;
        public static final double Y_RATE_LIMIT = 6.0;
        public static final double ROTATION_RATE_LIMIT = 5.0 * PI;

        public static final double HEADING_MAX_VELOCITY = PI * 2;
        public static final double HEADING_MAX_ACCELERATION = PI * 2;

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

    /**
     * Physical location of the camera on the robot, relative to the center of the
     * robot.
     */
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(
        // Left Camera
        // new Translation3d(Units.inchesToMeters(12.21), Units.inchesToMeters(-6.5615),
        // Units.inchesToMeters(-30.0)),
        // new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(-45)));
        // Right Camera
        new Translation3d(Units.inchesToMeters(12.21), Units.inchesToMeters(6.5615), Units.inchesToMeters(-30.0)),
        new Rotation3d(0, Math.toRadians(10.62), Math.toRadians(45)));
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
  }

    public static class AutoConstants {
        public static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(PI, 2 / PI);
        public static double THETA_kP = 2;
        public static double THETA_kI = 0.0;
        public static double THETA_kD = 0.0;

        public static double X_kP = 5;
        public static double X_kI = 0.0;
        public static double X_kD = 0.0;

        public static double Y_kP = 5;
        public static double Y_kI = 0.0;
        public static double Y_kD = 0.0;

        public static PIDController translationController = new PIDController(Constants.AutoConstants.X_kP,
                Constants.AutoConstants.X_kI, Constants.AutoConstants.X_kD);
        public static PIDController strafeController = new PIDController(Constants.AutoConstants.Y_kP,
                Constants.AutoConstants.Y_kI, Constants.AutoConstants.Y_kD);
        public static PIDController thetaController = new PIDController(Constants.AutoConstants.THETA_kP,
                Constants.AutoConstants.THETA_kI, Constants.AutoConstants.THETA_kD);

    }

    public static class PneumaticsConstants {
        public static final double MIN_PRESSURE = 80;
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
        public static final double ARM_ANGLE_ABSOLUTE_OFFSET = 62.42;
        public static final double MAX_ARM_LENGTH = Units.inchesToMeters(40);
        public static final double MIN_ARM_LENGTH = Units.inchesToMeters(10);
        /**
         * Tolerance for when the elevator considers itself to be at the target point
         */
        public static final double AT_TARGET_TOLERANCE = Units.inchesToMeters(0.5);
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
        public static final double AT_TARGET_TOLERANCE = Units.inchesToMeters(0.5);
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

    // These are all very arbitrary and need to be tuned
    public static class ArmSetpoints {
        /**
         * Put arm on ground to pick up a cone that's lying down
         */
        public static final ArmSetpoint GROUND_CONE_PICKUP = new ArmSetpoint(-20, 0, 20);
        /**
         * Put arm above ground to pick up a cone that's standing up
         */
        public static final ArmSetpoint ABOVE_CONE_PICKUP = new ArmSetpoint(-10, Units.inchesToMeters(10), -30);
        /**
         * Put arm down onto the cone that's standing up to pick it up
         */
        public static final ArmSetpoint ABOVE_CONE_PICKUP_2 = new ArmSetpoint(-15, Units.inchesToMeters(10), -25);
        /**
         * Put arm into a safe position for transit across the field
         */
        public static final ArmSetpoint TRANSIT = new ArmSetpoint(60, Units.inchesToMeters(0), 45);
        /**
         * Extend the arm out to place cone on the high peg
         */
        public static final ArmSetpoint HIGH_PEG = new ArmSetpoint(30, Units.inchesToMeters(40), -60);
        /**
         * Extend the arm out to place cone on the mid peg
         */
        public static final ArmSetpoint MID_PEG = new ArmSetpoint(30, Units.inchesToMeters(25), -60);
        /**
         * Extend the arm out to place on the hybrid node
         */
        public static final ArmSetpoint HYBRID_NODE = new ArmSetpoint(-5, Units.inchesToMeters(10), -60);
        /**
         * Extend the arm out to place cube on the high node
         */
        public static final ArmSetpoint HIGH_NODE = new ArmSetpoint(30, Units.inchesToMeters(35), -10);
        /**
         * Extend the arm out to place cube on the mid node
         */
        public static final ArmSetpoint MID_NODE = new ArmSetpoint(30, Units.inchesToMeters(20), -10);
        /**
         * Put arm out upside down to pick up a cone direct from the single substation
         */
        public static final ArmSetpoint SINGLE_SUBSTATION_PICKUP = new ArmSetpoint(0, Units.inchesToMeters(0), 45);
    }

}
