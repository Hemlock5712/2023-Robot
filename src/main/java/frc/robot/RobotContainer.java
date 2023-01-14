// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ChaseTagCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FieldHeadingDriveCommand;
import frc.robot.commands.WPIAStar;
import frc.robot.pathfind.Edge;
import frc.robot.pathfind.Node;
import frc.robot.pathfind.Obstacle;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.util.CustomAutoBuilder;
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
  // Set IP to 10.57.12.11
  // Set RoboRio to 10.57.12.2
  private final PhotonCamera photonCamera = new PhotonCamera("photonvision");

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(photonCamera, drivetrainSubsystem);

  private final ChaseTagCommand chaseTagCommand = new ChaseTagCommand(photonCamera, drivetrainSubsystem,
      poseEstimator::getCurrentPose);

  VisGraph AStarMap = new VisGraph();
  final Node finalNode = new Node(4, 4, Rotation2d.fromDegrees(180));
  //final List<Obstacle> obstacles = new ArrayList<Obstacle>();
  final List<Obstacle> obstacles = Constants.FieldConstants.obstacles;
  CustomAutoBuilder autoBuilder;

  HashMap<String, Command> eventMap = new HashMap<>();

  private final FieldHeadingDriveCommand fieldHeadingDriveCommand = new FieldHeadingDriveCommand(
      drivetrainSubsystem,
      () -> poseEstimator.getCurrentPose().getRotation(),
      () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -controller.getRightY(),
      () -> -controller.getRightX());

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
        drivetrainSubsystem,
        () -> poseEstimator.getCurrentPose().getRotation(),
        () -> -modifyAxis(controller.getLeftY()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getLeftX()) * DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(controller.getRightX()) * DrivetrainConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2));

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();

    AStarMap.addNode(finalNode);
    //SetUp AStar Map
    
    for(int i = 0; i<obstacles.size(); i++){
      System.out.println(obstacles.get(i));
      Constants.FieldConstants.obstacles.get(i).offset(0.5).addNodes(AStarMap);
    }

    for(int i = 0; i<AStarMap.getNodeSize();i++){
      Node startNode = AStarMap.getNode(i);
      System.out.println(""+startNode.getX()+","+startNode.getY());
      for(int j = i+1; j<AStarMap.getNodeSize(); j++){
        AStarMap.addEdge(new Edge(startNode, AStarMap.getNode(j)), obstacles);
      }
    }

    
    //Obstacle o = new Obstacle(new double[]{ 0, 0, 4, 4}, new double[] {0, 4, 4, 0});
    //Obstacle offset = o.offset(0.5f);
    //offset.addNodes(AStarMap);

    

    autoBuilder = new CustomAutoBuilder(
      poseEstimator::getCurrentPose,
      poseEstimator::setCurrentPose,
      Constants.DrivetrainConstants.KINEMATICS,
      new PIDConstants(.3, 0, 0),
      new PIDConstants(-3, 0, 0),
      drivetrainSubsystem::setModuleStates,
      eventMap,
      drivetrainSubsystem
    );
  }

  private void configureDashboard() {

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
    // Start button reseeds the steer motors to fix dead wheel
    controller.start().onTrue(Commands.runOnce(drivetrainSubsystem::reseedSteerMotorOffsets, drivetrainSubsystem));
    
    // Back button resets the robot pose
    controller.back().onTrue(Commands.runOnce(poseEstimator::resetFieldPosition, drivetrainSubsystem));

    controller.b().whileTrue(chaseTagCommand);

    controller.start().toggleOnTrue(fieldHeadingDriveCommand);

    controller.a().onTrue(Commands.runOnce(() -> poseEstimator.initializeGyro(0), drivetrainSubsystem));

    controller.y()
        .whileTrue(new WPIAStar(drivetrainSubsystem, poseEstimator, 
        new TrajectoryConfig(2, 2), 
        finalNode, obstacles, AStarMap));
    // controller.x().whileTrue(new DriveWithPathPlanner(drivetrainSubsystem,
    // poseEstimator, new PathConstraints(2, 2),
    // new PathPoint(new Translation2d(2.33, 2.03),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270)),
    // new PathPoint(new Translation2d(3, 3),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(180)),
    // new PathPoint(new Translation2d(2, 2),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270)),
    // new PathPoint(new Translation2d(Units.inchesToMeters(200), 2.03),
    // drivetrainSubsystem.getGyroscopeRotation(), Rotation2d.fromDegrees(270))));
    // controller.x().
    //     whileTrue(new PPAStar(drivetrainSubsystem, poseEstimator, 
    //         new PathConstraints(2, 2), finalNode, obstacles, AStarMap));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return autoBuilder.fullAuto(PathPlanner.loadPathGroup("Test Auto", new PathConstraints(1, 1)));
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = MathUtil.applyDeadband(value, DEADBAND);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
