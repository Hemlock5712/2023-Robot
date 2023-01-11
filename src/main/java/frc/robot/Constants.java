// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import java.util.List;
import java.util.Map;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.pathfind.Obstacle;
import frc.robot.swerve.ModuleConfiguration;

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

    public static final boolean ADD_TO_DASHBOARD = true;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.6;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.749;

    //Pick the longest side of the robot for this and measure outside bumper to outside bumper
    public static final double ROBOT_LENGTH_WIDTH = 0.749;

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2;
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 9;
    // public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(188.7);
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -toRadians(7.734375+180);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 3;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
    // public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(340.83);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -toRadians(162.5976+180);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 6;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 11;




    
    // public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(270);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -toRadians(88.9453125+180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    // public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(225);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -toRadians(44.384765625+180);
    
    public static final int PIGEON_ID = 13;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        ModuleConfiguration.MK4I_L2.getDriveReduction() *
        ModuleConfiguration.MK4I_L2.getWheelDiameter() * PI;

     /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 
        (DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0));

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double DRIVE_kS = 0.6716;
    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double DRIVE_kV = 2.5913;
    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
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
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(8, 8);

    public static final ProfiledPIDController xController = new ProfiledPIDController(2, 0, 0, X_CONSTRAINTS);
    public static final ProfiledPIDController yController = new ProfiledPIDController(2, 0, 0, Y_CONSTRAINTS);
    public static final ProfiledPIDController omegaController = new ProfiledPIDController(1.5, 0, 0, OMEGA_CONSTRAINTS);

  }

  public static class VisionConstants {

    /**
     * Physical location of the camera on the robot, relative to the center of the robot.
     */
    public static final Transform3d CAMERA_TO_ROBOT =
        new Transform3d(new Translation3d(-0.4, 0.2, 0.0), new Rotation3d());
    public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();
  }

  public static class AutoConstants {
    public static TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(PI, 2 / PI);
    public static double THETA_kP = 1.2;
    public static double THETA_kI = 0.0;
    public static double THETA_kD = 0.0;
    
    public static double X_kP = 1.2;
    public static double X_kI = 0.0;
    public static double X_kD = 0.0;

    public static double Y_kP = 1.2;
    public static double Y_kI = 0.0;
    public static double Y_kD = 0.0;

    public static PIDController m_translationController = new PIDController(Constants.AutoConstants.X_kP, Constants.AutoConstants.X_kI, Constants.AutoConstants.X_kD);
    public static PIDController m_strafeController = new PIDController(Constants.AutoConstants.Y_kP, Constants.AutoConstants.Y_kI, Constants.AutoConstants.Y_kD);
    public static PIDController m_thetaController = new PIDController(Constants.AutoConstants.THETA_kP, Constants.AutoConstants.THETA_kI, Constants.AutoConstants.THETA_kD);

  }

  public static final class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(651.25);
    public static final double fieldWidth = Units.inchesToMeters(315.5);
    public static final double tapeWidth = Units.inchesToMeters(2.0);

    // Dimensions for community and charging station, including the tape.
    public static final class Community {
        // Region dimensions
        public static final double innerX = 0.0;
        public static final double midX = Units.inchesToMeters(132.375); // Tape to the left of charging station
        public static final double outerX = Units.inchesToMeters(193.25); // Tape to the right of charging station
        public static final double leftY = Units.feetToMeters(18.0);
        public static final double midY = leftY - Units.inchesToMeters(59.39) + tapeWidth;
        public static final double rightY = 0.0;
        public static final Translation2d[] regionCorners = new Translation2d[] {
                new Translation2d(innerX, rightY),
                new Translation2d(innerX, leftY),
                new Translation2d(midX, leftY),
                new Translation2d(midX, midY),
                new Translation2d(outerX, midY),
                new Translation2d(outerX, rightY),
        };

        // Charging station dimensions
        public static final double chargingStationLength = Units.inchesToMeters(76.125);
        public static final double chargingStationWidth = Units.inchesToMeters(97.25);
        public static final double chargingStationOuterX = outerX - tapeWidth;
        public static final double chargingStationInnerX = chargingStationOuterX - chargingStationLength;
        public static final double chargingStationLeftY = midY - tapeWidth;
        public static final double chargingStationRightY = chargingStationLeftY - chargingStationWidth;
        public static final Translation2d[] chargingStationCorners = new Translation2d[] {
                new Translation2d(chargingStationInnerX, chargingStationRightY),
                new Translation2d(chargingStationInnerX, chargingStationLeftY),
                new Translation2d(chargingStationOuterX, chargingStationRightY),
                new Translation2d(chargingStationOuterX, chargingStationLeftY)
        };

        // Cable bump
        public static final double cableBumpInnerX = innerX + Grids.outerX + Units.inchesToMeters(95.25);
        public static final double cableBumpOuterX = cableBumpInnerX + Units.inchesToMeters(7);
        public static final Translation2d[] cableBumpCorners = new Translation2d[] {
                new Translation2d(cableBumpInnerX, 0.0),
                new Translation2d(cableBumpInnerX, chargingStationRightY),
                new Translation2d(cableBumpOuterX, 0.0),
                new Translation2d(cableBumpOuterX, chargingStationRightY)
        };
    }

    // Dimensions for grids and nodes
    public static final class Grids {
        // X layout
        public static final double outerX = Units.inchesToMeters(54.25);
        public static final double lowX = outerX - (Units.inchesToMeters(14.25) / 2.0); // Centered when under cube
                                                                                        // nodes
        public static final double midX = outerX - Units.inchesToMeters(22.75);
        public static final double highX = outerX - Units.inchesToMeters(39.75);

        // Y layout
        public static final int nodeRowCount = 9;
        public static final double nodeFirstY = Units.inchesToMeters(20.19);
        public static final double nodeSeparationY = Units.inchesToMeters(22.0);

        // Z layout
        public static final double cubeEdgeHigh = Units.inchesToMeters(3.0);
        public static final double highCubeZ = Units.inchesToMeters(35.5) - cubeEdgeHigh;
        public static final double midCubeZ = Units.inchesToMeters(23.5) - cubeEdgeHigh;
        public static final double highConeZ = Units.inchesToMeters(46.0);
        public static final double midConeZ = Units.inchesToMeters(34.0);

        // Translations (all nodes in the same column/row have the same X/Y coordinate)
        public static final Translation2d[] lowTranslations = new Translation2d[nodeRowCount];
        public static final Translation2d[] midTranslations = new Translation2d[nodeRowCount];
        public static final Translation3d[] mid3dTranslations = new Translation3d[nodeRowCount];
        public static final Translation2d[] highTranslations = new Translation2d[nodeRowCount];
        public static final Translation3d[] high3dTranslations = new Translation3d[nodeRowCount];

        static {
            for (int i = 0; i < nodeRowCount; i++) {
                boolean isCube = i == 1 || i == 4 || i == 7;
                lowTranslations[i] = new Translation2d(lowX, nodeFirstY + nodeSeparationY * i);
                midTranslations[i] = new Translation2d(midX, nodeFirstY + nodeSeparationY * i);
                mid3dTranslations[i] = new Translation3d(midX, nodeFirstY + nodeSeparationY * i,
                        isCube ? midCubeZ : midConeZ);
                high3dTranslations[i] = new Translation3d(
                        highX, nodeFirstY + nodeSeparationY * i, isCube ? highCubeZ : highConeZ);
                highTranslations[i] = new Translation2d(highX, nodeFirstY + nodeSeparationY * i);
            }
        }

        // Complex low layout (shifted to account for cube vs cone rows and wide edge
        // nodes)
        public static final double complexLowXCones = outerX - Units.inchesToMeters(16.0) / 2.0; // Centered X under
                                                                                                 // cone nodes
        public static final double complexLowXCubes = lowX; // Centered X under cube nodes
        public static final double complexLowOuterYOffset = nodeFirstY - Units.inchesToMeters(3.0)
                - (Units.inchesToMeters(25.75) / 2.0);

        public static final Translation2d[] complexLowTranslations = new Translation2d[] {
                new Translation2d(complexLowXCones, nodeFirstY - complexLowOuterYOffset),
                new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 1),
                new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 2),
                new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 3),
                new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 4),
                new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 5),
                new Translation2d(complexLowXCones, nodeFirstY + nodeSeparationY * 6),
                new Translation2d(complexLowXCubes, nodeFirstY + nodeSeparationY * 7),
                new Translation2d(
                        complexLowXCones, nodeFirstY + nodeSeparationY * 8 + complexLowOuterYOffset),
        };
    }

    // Dimensions for loading zone and substations, including the tape
    public static final class LoadingZone {
        // Region dimensions
        public static final double width = Units.inchesToMeters(99.0);
        public static final double innerX = FieldConstants.fieldLength;
        public static final double midX = fieldLength - Units.inchesToMeters(132.25);
        public static final double outerX = fieldLength - Units.inchesToMeters(264.25);
        public static final double leftY = FieldConstants.fieldWidth;
        public static final double midY = leftY - Units.inchesToMeters(50.5);
        public static final double rightY = leftY - width;
        public static final Translation2d[] regionCorners = new Translation2d[] {
                new Translation2d(
                        midX, rightY), // Start at lower left next to border with opponent community
                new Translation2d(midX, midY),
                new Translation2d(outerX, midY),
                new Translation2d(outerX, leftY),
                new Translation2d(innerX, leftY),
                new Translation2d(innerX, rightY),
        };

        // Double substation dimensions
        public static final double doubleSubstationLength = Units.inchesToMeters(14.0);
        public static final double doubleSubstationX = innerX - doubleSubstationLength;
        public static final double doubleSubstationShelfZ = Units.inchesToMeters(37.375);

        // Single substation dimensions
        public static final double singleSubstationWidth = Units.inchesToMeters(22.75);
        public static final double singleSubstationLeftX = FieldConstants.fieldLength - doubleSubstationLength
                - Units.inchesToMeters(88.77);
        public static final double singleSubstationCenterX = singleSubstationLeftX + (singleSubstationWidth / 2.0);
        public static final double singleSubstationRightX = singleSubstationLeftX + singleSubstationWidth;
        public static final Translation2d singleSubstationTranslation = new Translation2d(singleSubstationCenterX,
                leftY);

        public static final double singleSubstationHeight = Units.inchesToMeters(18.0);
        public static final double singleSubstationLowZ = Units.inchesToMeters(27.125);
        public static final double singleSubstationCenterZ = singleSubstationLowZ + (singleSubstationHeight / 2.0);
        public static final double singleSubstationHighZ = singleSubstationLowZ + singleSubstationHeight;
    }

    // Locations of staged game pieces
    public static final class StagingLocations {
        public static final double centerOffsetX = Units.inchesToMeters(47.36);
        public static final double positionX = fieldLength / 2.0 - Units.inchesToMeters(47.36);
        public static final double firstY = Units.inchesToMeters(36.19);
        public static final double separationY = Units.inchesToMeters(48.0);
        public static final Translation2d[] translations = new Translation2d[4];

        static {
            for (int i = 0; i < translations.length; i++) {
                translations[i] = new Translation2d(positionX, firstY + (i * separationY));
            }
        }
    }

    // AprilTag locations (do not flip for red alliance)
    public static final Map<Integer, Pose3d> aprilTags = Map.of(
            1,
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            2,
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            3,
            new Pose3d(
                    Units.inchesToMeters(610.77),
                    Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                    Units.inchesToMeters(18.22),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            4,
            new Pose3d(
                    Units.inchesToMeters(636.96),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d(0.0, 0.0, Math.PI)),
            5,
            new Pose3d(
                    Units.inchesToMeters(14.25),
                    Units.inchesToMeters(265.74),
                    Units.inchesToMeters(27.38),
                    new Rotation3d()),
            6,
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(174.19), // FIRST's diagram has a typo (it says 147.19)
                    Units.inchesToMeters(18.22),
                    new Rotation3d()),
            7,
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(108.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()),
            8,
            new Pose3d(
                    Units.inchesToMeters(40.45),
                    Units.inchesToMeters(42.19),
                    Units.inchesToMeters(18.22),
                    new Rotation3d()));

    public static List<Obstacle> obstacles = List.of(
            // Blue Charging Station
            new Obstacle(new double[] {
                    FieldConstants.Community.chargingStationCorners[0].getX(),
                    FieldConstants.Community.chargingStationCorners[1].getX(),
                    FieldConstants.Community.chargingStationCorners[2].getX(),
                    FieldConstants.Community.chargingStationCorners[3].getX(),
            }, new double[] {
                    FieldConstants.Community.chargingStationCorners[0].getY(),
                    FieldConstants.Community.chargingStationCorners[1].getY(),
                    FieldConstants.Community.chargingStationCorners[2].getY(),
                    FieldConstants.Community.chargingStationCorners[3].getY(),
            }),
            // Red Charging Station
            new Obstacle(new double[] {
                    allianceFlip(FieldConstants.Community.chargingStationCorners[0]).getX(),
                    allianceFlip(FieldConstants.Community.chargingStationCorners[1]).getX(),
                    allianceFlip(FieldConstants.Community.chargingStationCorners[2]).getX(),
                    allianceFlip(FieldConstants.Community.chargingStationCorners[3]).getX(),
            }, new double[] {
                    allianceFlip(FieldConstants.Community.chargingStationCorners[0]).getY(),
                    allianceFlip(FieldConstants.Community.chargingStationCorners[1]).getY(),
                    allianceFlip(FieldConstants.Community.chargingStationCorners[2]).getY(),
                    allianceFlip(FieldConstants.Community.chargingStationCorners[3]).getY(),
            }));

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with
     * the origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Translation2d allianceFlip(Translation2d translation) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Translation2d(fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    /**
     * Flips a pose to the correct side of the field based on the current alliance
     * color. By default,
     * all translations and poses in {@link FieldConstants} are stored with the
     * origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Pose2d allianceFlip(Pose2d pose) {
        if (DriverStation.getAlliance() == Alliance.Red) {
            return new Pose2d(
                    fieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }
  }
}
