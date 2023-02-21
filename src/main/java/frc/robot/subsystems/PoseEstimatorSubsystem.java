package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.util.FieldConstants;

public class PoseEstimatorSubsystem extends SubsystemBase {

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much
  // you trust your various sensors. Smaller numbers will cause the filter to
  // "trust" the estimate from that particular component more than the others.
  // This in turn means the particualr component will have a stronger influence
  // on the final pose estimate.

  /**
   * Standard deviations of model states. Increase these numbers to trust your
   * model's state estimates less. This
   * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
   * meters.
   */
  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);

  /**
   * Standard deviations of the vision measurements. Increase these numbers to
   * trust global measurements from vision
   * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
   * radians.
   */
  private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, 0.9);

  private final DrivetrainSubsystem drivetrainSubsystem;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final PhotonPoseEstimator photonPoseEstimator;

  private double previousPipelineTimestamp = 0;
  private OriginPosition originPosition = OriginPosition.kBlueAllianceWallRightSide;

  private final ArrayList<Double> xValues = new ArrayList<Double>();
  private final ArrayList<Double> yValues = new ArrayList<Double>();

  public PoseEstimatorSubsystem(PhotonCamera photonCamera, DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    PhotonPoseEstimator photonPoseEstimator;
    try {
      var layout = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
      layout.setOrigin(originPosition);
      photonPoseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP, photonCamera,
          Constants.VisionConstants.ROBOT_TO_CAMERA);
      photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } catch (IOException e) {
      DriverStation.reportError("Failed to load AprilTagFieldLayout", e.getStackTrace());
      photonPoseEstimator = null;
    }
    this.photonPoseEstimator = photonPoseEstimator;

    ShuffleboardTab tab = Shuffleboard.getTab("Vision");

    poseEstimator = new SwerveDrivePoseEstimator(
        DrivetrainConstants.KINEMATICS,
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        new Pose2d(),
        stateStdDevs,
        visionMeasurementStdDevs);

    tab.addString("Pose", this::getFomattedPose).withPosition(0, 0).withSize(2, 0);
    tab.add("Field", field2d).withPosition(2, 0).withSize(6, 4);
  }

  /**
   * Sets the alliance. This is used to configure the origin of the AprilTag map
   * 
   * @param alliance alliance
   */
  public void setAlliance(Alliance alliance) {
    var fieldTags = photonPoseEstimator.getFieldTags();
    boolean allianceChanged = false;
    switch (alliance) {
      case Blue:
        fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
        allianceChanged = (originPosition == OriginPosition.kRedAllianceWallRightSide);
        originPosition = OriginPosition.kBlueAllianceWallRightSide;
        break;
      case Red:
        fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
        allianceChanged = (originPosition == OriginPosition.kBlueAllianceWallRightSide);
        originPosition = OriginPosition.kRedAllianceWallRightSide;
        break;
      default:
        // No valid alliance data. Nothing we can do about it
    }
    if (allianceChanged) {
      // The alliance changed, which changes the coordinate system.
      // Since a tag may have been seen and the tags are all relative to the
      // coordinate system, the estimated pose
      // needs to be transformed to the new coordinate system.
      var newPose = flipAlliance(poseEstimator.getEstimatedPosition());
      setCurrentPose(newPose);
    }
  }

  @Override
  public void periodic() {
    // Update pose estimator with drivetrain sensors
    poseEstimator.update(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions());

    // Conversion so robot appears where it actually is on field instead of always
    // on blue.
    // xValues.add(getCurrentPose().getX());
    // yValues.add(getCurrentPose().getY());
    // double xAverage = xValues.stream().mapToDouble(a ->
    // a).average().getAsDouble();
    // double yAverage = yValues.stream().mapToDouble(a ->
    // a).average().getAsDouble();
    // double summation = 0.0;
    // for (int i = 0; i < xValues.size(); i++) {
    // summation += (Math.pow(xValues.get(i) - xAverage, 2) +
    // Math.pow(yValues.get(i) - yAverage, 2));
    // }
    // double RMS = Math.sqrt((1.0 / (double) xValues.size() * summation));
    // System.out.println("RMS: " + RMS);

    if (photonPoseEstimator != null) {
      // Update pose estimator with the best visible target
      photonPoseEstimator.update().ifPresent(estimatedRobotPose -> {
        var estimatedPose = estimatedRobotPose.estimatedPose;
        // Make sure we have a new measurement, and that it's on the field
        if (estimatedRobotPose.timestampSeconds != previousPipelineTimestamp
            && estimatedPose.getX() > 0.0 && estimatedPose.getX() <= FieldConstants.FIELD_LENGTH_METERS
            && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= FieldConstants.FIELD_WIDTH_METERS) {
          previousPipelineTimestamp = estimatedRobotPose.timestampSeconds;
          poseEstimator.addVisionMeasurement(estimatedPose.toPose2d(), estimatedRobotPose.timestampSeconds);
        }
      });
    }

    Pose2d dashboardPose = getCurrentPose();
    if (originPosition == OriginPosition.kRedAllianceWallRightSide) {
      // Flip the pose when red, since the dashboard field photo cannot be rotated
      dashboardPose = flipAlliance(dashboardPose);
    }
    field2d.setRobotPose(dashboardPose);
  }

  private String getFomattedPose() {
    var pose = getCurrentPose();
    return String.format("(%.2f, %.2f) %.2f degrees",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * 
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPosition(
        drivetrainSubsystem.getGyroscopeRotation(),
        drivetrainSubsystem.getModulePositions(),
        newPose);
  }

  /**
   * Resets the position on the field to 0,0 0-degrees, with forward being
   * downfield. This resets
   * what "forward" is for field oriented driving.
   */
  public void resetFieldPosition() {
    setCurrentPose(new Pose2d());
  }

  /**
   * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
   * always on the right corner of your
   * alliance wall, so for 2023, the field elements are at different coordinates
   * for each alliance.
   * 
   * @param poseToFlip pose to transform to the other alliance
   * @return pose relative to the other alliance's coordinate system
   */
  private Pose2d flipAlliance(Pose2d poseToFlip) {
    return poseToFlip.relativeTo(new Pose2d(
        new Translation2d(FieldConstants.FIELD_LENGTH_METERS, FieldConstants.FIELD_WIDTH_METERS),
        new Rotation2d(Math.PI)));
  }

  public void addTrajectory(PathPlannerTrajectory traj) {
    field2d.getObject("Trajectory").setTrajectory(traj);
  }

  /**
   * Resets the holonomic rotation of the robot (gyro last year)
   * This would be used if Apriltags are not getting accurate pose estimation
   */
  public void resetHolonomicRotation() {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(0),
        drivetrainSubsystem.getModulePositions(),
        getCurrentPose());
  }

  public void resetPoseRating() {
    xValues.clear();
    yValues.clear();
  }

}