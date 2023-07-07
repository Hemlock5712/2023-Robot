package frc.robot.commands;

import static frc.robot.Constants.AutoConstants.THETA_kD;
import static frc.robot.Constants.AutoConstants.THETA_kI;
import static frc.robot.Constants.AutoConstants.THETA_kP;
import static frc.robot.Constants.DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.Y_RATE_LIMIT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Command for teleop driving where translation is field oriented and rotation
 * velocity is controlled by the driver.
 * 
 * Translation is specified on the field-relative coordinate system. The Y-axis
 * runs parallel to the alliance wall, left
 * is positive. The X-axis runs down field toward the opposing alliance wall,
 * away from the alliance wall is positive.
 */
public class FieldOrientedDriveCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrainSubsystem;
  private final Supplier<Rotation2d> robotAngleSupplier;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  private final SlewRateLimiter translateXRateLimiter = new SlewRateLimiter(X_RATE_LIMIT);
  private final SlewRateLimiter translateYRateLimiter = new SlewRateLimiter(Y_RATE_LIMIT);
  private final SlewRateLimiter rotationRateLimiter = new SlewRateLimiter(ROTATION_RATE_LIMIT);
  private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
  private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.4,
      MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
  private static ProfiledPIDController thetaController;
  private static ProfiledPIDController thetaDoubleController;

  private Trigger isAligningSingleSubTrigger;
  private Trigger isEvadingTrigger;

  private static double TARGET_ANGELE = 1.5708;
  private static double ALLIANCE_SPECIFIC = TARGET_ANGELE;

  /**
   * Constructor
   * 
   * @param drivetrainSubsystem  drivetrain
   * @param robotAngleSupplier   supplier for the current angle of the robot
   * @param translationXSupplier supplier for translation X component, in meters
   *                             per second
   * @param translationYSupplier supplier for translation Y component, in meters
   *                             per second
   * @param rotationSupplier     supplier for rotation component, in radians per
   *                             second
   */
  public FieldOrientedDriveCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      Supplier<Rotation2d> robotAngleSupplier,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier,
      Trigger isAligningSingleSubTrigger,
      Trigger isEvadingTrigger) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.isAligningSingleSubTrigger = isAligningSingleSubTrigger;
    this.isEvadingTrigger = isEvadingTrigger;

    thetaController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, DEFAULT_OMEGA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(THETA_TOLERANCE);
    thetaDoubleController = new ProfiledPIDController(THETA_kP, THETA_kI, THETA_kD, DEFAULT_OMEGA_CONSTRAINTS);
    thetaDoubleController.enableContinuousInput(-Math.PI, Math.PI);
    thetaDoubleController.setTolerance(THETA_TOLERANCE);

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    var robotAngle = robotAngleSupplier.get();
    thetaController.reset(robotAngle.getRadians());
    if (Constants.DrivetrainConstants.alliance == DriverStation.Alliance.Red) {
      ALLIANCE_SPECIFIC = -1 * TARGET_ANGELE;
    } else {
      ALLIANCE_SPECIFIC = TARGET_ANGELE;
    }
    thetaController.setGoal(ALLIANCE_SPECIFIC);
    thetaDoubleController.setGoal(0);

    // Calculate field relative speeds
    var chassisSpeeds = drivetrainSubsystem.getChassisSpeeds();
    var fieldSpeeds = getFieldSpeeds(chassisSpeeds, robotAngle);
    var robotSpeeds = getRobotSpeeds(fieldSpeeds, chassisSpeeds);

    // Reset the slew rate limiters, in case the robot is already moving
    translateXRateLimiter.reset(robotSpeeds.vxMetersPerSecond);
    translateYRateLimiter.reset(robotSpeeds.vyMetersPerSecond);
    rotationRateLimiter.reset(robotSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    var robotPose = robotAngleSupplier.get();
    if (isAligningSingleSubTrigger.getAsBoolean() && !drivetrainSubsystem.isDoubleSub()) {
      var omegaSpeed = thetaController.calculate(robotPose.getRadians());
      if (thetaController.atGoal()) {
        omegaSpeed = 0;
      }
      drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
              translateYRateLimiter.calculate(translationYSupplier.getAsDouble()), omegaSpeed,
              robotAngleSupplier.get()));
    } else if (isEvadingTrigger.getAsBoolean()) {
      drivetrainSubsystem.drive(
          new Translation2d(translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
              translateYRateLimiter.calculate(translationYSupplier.getAsDouble())),
          rotationRateLimiter.calculate(rotationSupplier.getAsDouble()),
          robotAngleSupplier.get());
    } else if (isAligningSingleSubTrigger.getAsBoolean() && drivetrainSubsystem.isDoubleSub()) {
      var omegaSpeed = thetaDoubleController.calculate(robotPose.getRadians());
      if (thetaDoubleController.atGoal()) {
        omegaSpeed = 0;
      }
      drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
              translateYRateLimiter.calculate(translationYSupplier.getAsDouble()), omegaSpeed,
              robotAngleSupplier.get()));
    } else {
      drivetrainSubsystem.drive(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translateXRateLimiter.calculate(translationXSupplier.getAsDouble()),
              translateYRateLimiter.calculate(translationYSupplier.getAsDouble()),
              rotationRateLimiter.calculate(rotationSupplier.getAsDouble()),
              robotAngleSupplier.get()));
    }

  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  public static Translation2d getFieldSpeeds(ChassisSpeeds chassisSpeeds, Rotation2d robotAngle) {
    return new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
        .rotateBy(robotAngle);
  }

  public static ChassisSpeeds getRobotSpeeds(Translation2d fieldSpeeds, ChassisSpeeds chassisSpeeds) {
    return new ChassisSpeeds(fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
  }

  public static void setAllianceAngle() {
    if (Constants.DrivetrainConstants.alliance == DriverStation.Alliance.Red) {
      ALLIANCE_SPECIFIC = -1 * TARGET_ANGELE;
    } else {
      ALLIANCE_SPECIFIC = TARGET_ANGELE;
    }
    thetaController.setGoal(ALLIANCE_SPECIFIC);
  }
}