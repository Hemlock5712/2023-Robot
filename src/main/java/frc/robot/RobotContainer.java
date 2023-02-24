// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.auto.PPSwerveFollower;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.driver.GoToLoad;
import frc.robot.commands.driver.GoToPlace;
import frc.robot.commands.operator.NextNode;
import frc.robot.pathfind.MapCreator;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.util.FieldConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);
  // Set IP to 10.57.12.11
  // Set RoboRio to 10.57.12.2
  private final PhotonCamera photonCamera = new PhotonCamera("photonvision");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);
  private final TestSubsystem testSubsystem = new TestSubsystem();
  private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(photonCamera, drivetrainSubsystem,
      poseEstimator::getCurrentPose);

  final List<Obstacle> standardObstacles = FieldConstants.standardObstacles;
  final List<Obstacle> cablePath = FieldConstants.cablePath;

  public MapCreator map = new MapCreator();
  public VisGraph standardMap = new VisGraph();
  public VisGraph cableMap = new VisGraph();

  private PneumaticHub pch = new PneumaticHub();

  HashMap<String, Command> eventMap = new HashMap<>();

  private final FieldHeadingDriveCommand fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
      drivetrainSubsystem,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -controller.getRightY(),
      () -> -controller.getRightX());

  private final FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
      drivetrainSubsystem,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2);

  private final Timer reseedTimer = new Timer();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(fieldOrientedDriveCommand);

    map.createGraph(standardMap, standardObstacles);
    map.createGraph(cableMap, cablePath);

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
    pch.enableCompressorAnalog(80, 120);
  }

  private void configureDashboard() {

  }

  public void periodic() {
    SmartDashboard.putNumber("PCH/Pressure", pch.getPressure(0));
    SmartDashboard.putNumber("PCH/MinPressure", Constants.PneumaticsConstants.MIN_PRESSURE);
    SmartDashboard.putNumber("PCH/MaxPressure", Constants.PneumaticsConstants.MAX_PRESSURE);
    SmartDashboard.putBoolean("PCH/IsRunning", pch.getCompressor());
  }

  public void disabledPeriodic() {
    // Reseed the motor offset continuously when the robot is disabled to help solve
    // dead wheel issue
    if (reseedTimer.advanceIfElapsed(1.0)) {
      drivetrainSubsystem.reseedSteerMotorOffsets();
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button resets the robot pose
    controller.back().onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));

    controller.b().whileTrue(chaseTagCommand);

    // controller.start().toggleOnTrue(fieldHeadingDriveCommand);

    controller.y().whileTrue(
        new GoToLoad(drivetrainSubsystem, poseEstimator, new PathConstraints(2, 2), standardObstacles, standardMap));
    controller.a().whileTrue(
        new GoToPlace(drivetrainSubsystem, poseEstimator, new PathConstraints(2, 2), standardObstacles, standardMap));

    controller.rightBumper().whileTrue(new RunIntakeCommand(testSubsystem));
    controller.leftBumper().whileTrue(new ReverseIntakeCommand(testSubsystem));
    controller.rightTrigger(.5).whileTrue(new OpenClaw(testSubsystem));

    // controller.a().onTrue(Commands.runOnce(poseEstimator::resetHolonomicRotation,
    // drivetrainSubsystem));

    // controller.a().onTrue(Commands.runOnce(poseEstimator::resetPoseRating));

    controller2.rightBumper().whileTrue(new NextNode(drivetrainSubsystem, 1));
    controller2.leftBumper().whileTrue(new NextNode(drivetrainSubsystem, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PPSwerveFollower(drivetrainSubsystem, poseEstimator, "New Path", new PathConstraints(2, 1), false);
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }

  public void onAllianceChanged(Alliance currentAlliance) {
    poseEstimator.setAlliance(currentAlliance);
  }
}
