package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.pathfind.Obstacle;

public final class FieldConstants {

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

  

    /**
     * Flips a translation to the correct side of the field based on the current
     * alliance color. By
     * default, all translations and poses in {@link FieldConstants} are stored with
     * the origin at the
     * rightmost point on the BLUE ALLIANCE wall.
     */
    public static Translation2d allianceFlip(Translation2d translation) {
      if (DriverStation.getAlliance() == Alliance.Red) {
          return new Translation2d(FieldConstants.fieldLength - translation.getX(), translation.getY());
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
                  FieldConstants.fieldLength - pose.getX(),
                  pose.getY(),
                  new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
      } else {
          return pose;
      }
  }

  public static List<Obstacle> obstacles = List.of(
    // Blue Charging Station
    new Obstacle(new double[] {
            FieldConstants.Community.chargingStationCorners[0].getX(),
            FieldConstants.Community.chargingStationCorners[1].getX(),
            FieldConstants.Community.chargingStationCorners[3].getX(),
            FieldConstants.Community.chargingStationCorners[2].getX(),
    }, new double[] {
            FieldConstants.Community.chargingStationCorners[0].getY(),
            FieldConstants.Community.chargingStationCorners[1].getY(),
            FieldConstants.Community.chargingStationCorners[3].getY(),
            FieldConstants.Community.chargingStationCorners[2].getY()
    })
    ,
    // Red Charging Station
    new Obstacle(new double[] {
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[2]).getX(),
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[3]).getX(),
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[1]).getX(),
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[0]).getX(),
    }, new double[] {
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[2]).getY(),
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[3]).getY(),
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[1]).getY(),
            FieldConstants.allianceFlip(FieldConstants.Community.chargingStationCorners[0]).getY()
    })
    );

}