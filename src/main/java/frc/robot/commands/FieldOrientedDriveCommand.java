package frc.robot.commands;

import static frc.robot.Constants.TeleopDriveConstants.ROTATION_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.X_RATE_LIMIT;
import static frc.robot.Constants.TeleopDriveConstants.Y_RATE_LIMIT;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private Trigger isEvadingTrigger;
  private boolean isEvading;

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
      Trigger isEvadingTrigger) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.robotAngleSupplier = robotAngleSupplier;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;
    this.isEvadingTrigger = isEvadingTrigger;
    this.isEvading = isEvadingTrigger.getAsBoolean();

    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    var robotAngle = robotAngleSupplier.get();

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

    double rAxis = rotationSupplier.getAsDouble();
    double xAxis = translationXSupplier.getAsDouble();
    double yAxis = translationYSupplier.getAsDouble();

    // /* Square joystick inputs */
    // double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
    // double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
    // double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;
    /* Filter joystick inputs using slew rate limiter */
    double yAxisFiltered = translateYRateLimiter.calculate(yAxis);
    double xAxisFiltered = translateXRateLimiter.calculate(xAxis);
    double rAxisFiltered = rotationRateLimiter.calculate(rAxis);
    this.isEvading = isEvadingTrigger.getAsBoolean();

    drivetrainSubsystem.drive(
        new Translation2d(xAxisFiltered, yAxisFiltered),
        rAxisFiltered,
        robotAngleSupplier.get(),
        this.isEvading);
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
}