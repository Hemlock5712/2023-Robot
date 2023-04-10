// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.TeleopDriveConstants.DEADBAND;

import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.FieldOrientedDriveCommand;
import frc.robot.commands.HoldIntakeCommand;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RunIntakeCommand;
import frc.robot.commands.balance.NewBalance;
import frc.robot.commands.driver.DriveToPoseCommand;
import frc.robot.commands.driver.WheelXMode;
import frc.robot.commands.operator.HighPlace;
import frc.robot.commands.operator.MidPlace;
import frc.robot.commands.operator.MoveToSetpoint;
import frc.robot.commands.operator.SingleSubstation;
import frc.robot.commands.operator.VisionController;
import frc.robot.commands.operator.subcommands.MoveSpacerAngle;
import frc.robot.pathfind.MapCreator;
import frc.robot.pathfind.VisGraph;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.SpacerSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.FieldConstants;
import frc.robot.util.PiecePicker;
import frc.robot.util.ResetGyro;
import frc.robot.util.enums.GamePiece;

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

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

  private final PoseEstimatorSubsystem poseEstimator = new PoseEstimatorSubsystem(
      drivetrainSubsystem::getGyroscopeRotation, drivetrainSubsystem::getModulePositions);
  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final ExtensionSubsystem extensionSubsystem = new ExtensionSubsystem();
  private final WristSubsystem wristSubsystem = new WristSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(ledSubsystem);
  private final SpacerSubsystem spacerSubsystem = new SpacerSubsystem();

  // final List<Obstacle> cablePath = FieldConstants.cablePath;

  public MapCreator map = new MapCreator();
  public VisGraph standardMap = new VisGraph();
  // public VisGraph cableMap = new VisGraph();

  private PneumaticHub pch = new PneumaticHub(1);

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  Map<String, Command> eventMap = Map.ofEntries(
      Map.entry(
          "cubeHigh",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.HIGH_CUBE)
              .alongWith(new RunIntakeCommand(intakeSubsystem))
              .withTimeout(1)),
      Map.entry(
          "cubeMid",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.MID_CUBE)
              .withTimeout(1)),
      Map.entry(
          "extendLow",
          new MoveToSetpoint(elevatorSubsystem,
              extensionSubsystem, wristSubsystem,
              Constants.ArmSetpoints.HYBRID_NODE)),
      Map.entry(
          "coneHigh",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.HIGH_PEG)
              .alongWith(new RunIntakeCommand(intakeSubsystem))
              .withTimeout(1)),
      Map.entry(
          "coneMid",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.MID_PEG)
              .alongWith(new RunIntakeCommand(intakeSubsystem))
              .withTimeout(1)),
      Map.entry(
          "outtake",
          new WaitCommand(0.25).deadlineWith(new ReverseIntakeCommand(intakeSubsystem))),
      Map.entry(
          "extendIn",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, new ArmSetpoint(30, 0, 45))
              .withTimeout(0.5)),
      Map.entry(
          "transit",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, Constants.ArmSetpoints.TRANSIT)
              .withTimeout(0.1)),
      Map.entry(
          "newAutoBalance",
          new NewBalance(drivetrainSubsystem, poseEstimator)),
      Map.entry(
          "goToIntakeMode",
          new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem,
              Constants.ArmSetpoints.GROUND_CUBE_PICKUP)),
      Map.entry(
          "startIntake",
          new WaitCommand(2).deadlineWith(new RunIntakeCommand(intakeSubsystem))),
      Map.entry(
          "cone",
          new InstantCommand(() -> {
            PiecePicker.toggle(false);
            ledSubsystem.setGamePiece(GamePiece.CONE);
          })),
      Map.entry(
          "visionOn",
          new InstantCommand(() -> {
            VisionController.setVisionOn();
          })),
      Map.entry(
          "visionOff",
          new InstantCommand(() -> {
            VisionController.setVisionOff();
          })),
      Map.entry(
          "cube",
          new InstantCommand(() -> {
            PiecePicker.toggle(true);
            ledSubsystem.setGamePiece(GamePiece.CUBE);
          })));

  // private final FieldHeadingDriveCommand fieldHeadingDriveCommand = new
  // FieldHeadingDriveCommand(
  // drivetrainSubsystem,
  // () -> poseEstimator.getCurrentPose().getRotation(),
  // () -> -modifyAxis(controller.getLeftY()) *
  // DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
  // () -> -modifyAxis(controller.getLeftX()) *
  // DrivetrainConstants.MAX_VELOCITY_METERS_PER_SECOND,
  // () -> -controller.getRightY(),
  // () -> -controller.getRightX());

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

    map.createGraph(standardMap, FieldConstants.standardObstacles);
    // map.createGraph(cableMap, cablePath);

    PiecePicker.toggle(false);
    intakeSubsystem.setDefaultCommand(new HoldIntakeCommand(intakeSubsystem));

    autoChooser.addOption("Wall Side 2.5 Cube Blue", makeAutoBuilderCommand("WALL25BLUE", new PathConstraints(2.5, 2)));
    // autoChooser.addOption("Wall Side 2.5 Cube Red",
    // makeAutoBuilderCommand("WALL25RED", new PathConstraints(2.5, 2)));

    autoChooser.addOption("Wall Side 2.5 Cube Park Blue",
        makeAutoBuilderCommand("WALL25PARKBLUE", new PathConstraints(2.5, 2)).withTimeout(14.9)
            .andThen(new WheelXMode(drivetrainSubsystem)));
    // autoChooser.addOption("Wall Side 2.5 Cube Park Red",
    // makeAutoBuilderCommand("WALL25PARKRED", new PathConstraints(2.5,
    // 2)).withTimeout(14.9)
    // .andThen(new WheelXMode(drivetrainSubsystem)));

    autoChooser.setDefaultOption("Human 2.5 Cube Park Blue",
        makeAutoBuilderCommand("HUMAN25PARKBLUE", new PathConstraints(3.5, 3)).withTimeout(14.9)
            .andThen(new WheelXMode(drivetrainSubsystem)));
    // autoChooser.addOption("Human 2.5 Cube Park Red",
    // makeAutoBuilderCommand("HUMAN25PARKRED", new PathConstraints(3.5,
    // 3)).withTimeout(14.9)
    // .andThen(new WheelXMode(drivetrainSubsystem)));

    autoChooser.addOption("Human 3 Cube Blue", makeAutoBuilderCommand("HUMAN3BLUE", new PathConstraints(3, 3))
        .withTimeout(14.9).andThen(new WaitCommand(0.25).deadlineWith(new ReverseIntakeCommand(intakeSubsystem))));
    // autoChooser.addOption("Human 3 Cube Red", makeAutoBuilderCommand("HUMAN3RED",
    // new PathConstraints(3, 3))
    // .withTimeout(14.9).andThen(new WaitCommand(0.25).deadlineWith(new
    // ReverseIntakeCommand(intakeSubsystem))));

    autoChooser.addOption("Human 2.5 Cube Blue", makeAutoBuilderCommand("HUMAN25BLUE", new PathConstraints(2, 2)));
    // autoChooser.addOption("Human 2.5 Cube Red",
    // makeAutoBuilderCommand("HUMAN25RED", new PathConstraints(3, 3)));

    // autoChooser.addOption("Human 3 Cube Center Blue",
    // makeAutoBuilderCommand("HUMAN3CENTERBLUE", new PathConstraints(3, 3)));
    // autoChooser.addOption("Human 3 Cube Center Red",
    // makeAutoBuilderCommand("HUMAN3CENTERRED", new PathConstraints(3, 3)));

    autoChooser.addOption("NO VISION", makeAutoBuilderCommand("EMERGENCY", new PathConstraints(2, 2)));

    autoChooser.addOption("Do Nothing", Commands.none());

    SmartDashboard.putData(autoChooser);

    // Configure the button bindings
    configureButtonBindings();
    configureDashboard();
    reseedTimer.start();
    pch.enableCompressorAnalog(Constants.PneumaticsConstants.MIN_PRESSURE, Constants.PneumaticsConstants.MAX_PRESSURE);
  }

  private void configureDashboard() {
    /**** Vision tab ****/
    final var visionTab = Shuffleboard.getTab("Vision");
    poseEstimator.addDashboardWidgets(visionTab);

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

    controller.leftBumper()
        .whileTrue(new DriveToPoseCommand(drivetrainSubsystem, poseEstimator::getCurrentPose, 14.35, ledSubsystem));

    controller.leftTrigger(0.5).whileTrue(new RunIntakeCommand(intakeSubsystem));

    controller.rightTrigger(0.5).whileTrue(new ReverseIntakeCommand(intakeSubsystem));

    controller.a().whileTrue(new InstantCommand(() -> {
      ResetGyro.resetGyro(poseEstimator);
    }));

    controller.rightBumper().whileTrue(new InstantCommand(() -> {
      PiecePicker.toggle(true);
      ledSubsystem.setGamePiece(GamePiece.CUBE);
    }).andThen(new MoveSpacerAngle(24, spacerSubsystem).withTimeout(0.5))
        .andThen(
            new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem,
                Constants.ArmSetpoints.GROUND_CUBE_PICKUP).alongWith(
                    new RunIntakeCommand(intakeSubsystem))));

    controller2.leftBumper().onTrue(new InstantCommand(() -> {
      PiecePicker.toggle(true);
      ledSubsystem.setGamePiece(GamePiece.CUBE);
    }));

    controller2.leftTrigger(0.5).whileTrue(
        new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, new ArmSetpoint(30, 0, 45))
            .withTimeout(0.5).alongWith(new MoveSpacerAngle(24, spacerSubsystem).withTimeout(3.0)).andThen(
                new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem,
                    Constants.ArmSetpoints.TRANSIT)));

    controller2.rightBumper().onTrue(new InstantCommand(() -> {
      PiecePicker.toggle(false);
      ledSubsystem.setGamePiece(GamePiece.CONE);
    }));

    controller2.a().whileTrue(new MoveToSetpoint(elevatorSubsystem,
        extensionSubsystem, wristSubsystem,
        Constants.ArmSetpoints.HYBRID_NODE));

    controller2.b().whileTrue(new MidPlace(elevatorSubsystem,
        extensionSubsystem, wristSubsystem));

    controller2.x()
        .whileTrue(new ParallelDeadlineGroup(new MoveSpacerAngle(2, spacerSubsystem),
            new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem, new ArmSetpoint(20, -0.06, 45)))
            .andThen(
                new SingleSubstation(elevatorSubsystem, extensionSubsystem, wristSubsystem, intakeSubsystem)));

    controller2.y().whileTrue(new HighPlace(elevatorSubsystem,
        extensionSubsystem, wristSubsystem));

    controller2.back().and(controller2.start()).whileTrue(new MoveToSetpoint(elevatorSubsystem, extensionSubsystem,
        wristSubsystem, Constants.ArmSetpoints.STARTING_CONFIG));

    // Drive to Point stuff beyond this point
    // controller2.pov(0).whileTrue(new NextNode(Direction.Up));
    // controller2.pov(90).whileTrue(new NextNode(Direction.Right));
    // controller2.pov(180).whileTrue(new NextNode(Direction.Down));
    // controller2.pov(270).whileTrue(new NextNode(Direction.Left));
  }

  public void startTeleopPosCommand() {
    double currentAngle = elevatorSubsystem.getAngle();
    double currentExtension = extensionSubsystem.getHeight();
    double wristAngle = wristSubsystem.getAngle();
    new MoveToSetpoint(elevatorSubsystem, extensionSubsystem, wristSubsystem,
        new ArmSetpoint(Math.toDegrees(currentAngle), currentExtension, wristAngle))
        .schedule();

    // elevatorSubsystem.setMotorVoltage(0);
    // extensionSubsystem.setMotorVoltage(0);
    // wristSubsystem.setVoltage(0);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    // return new AutonomousCenterBalance(drivetrainSubsystem, poseEstimator,
    // elevatorSubsystem, extensionSubsystem,
    // wristSubsystem, intakeSubsystem);
  }

  private CommandBase makeAutoBuilderCommand(String pathName, PathConstraints constraints) {
    var path = PathPlanner.loadPath(pathName, constraints);

    poseEstimator.addTrajectory(path);
    // controllerCommand = DrivetrainSubsystem.followTrajectory(driveSystem,
    // poseEstimatorSystem, alliancePath);
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        poseEstimator::getCurrentPose,
        poseEstimator::setCurrentPose,
        Constants.DrivetrainConstants.KINEMATICS,
        Constants.AutoConstants.translationConstants,
        Constants.AutoConstants.rotationConstants,
        drivetrainSubsystem::setModuleStates,
        eventMap,
        true,
        drivetrainSubsystem);
    return autoBuilder.fullAuto(path);

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

  public void changeVision(boolean useVision) {
    Constants.VisionConstants.USE_VISION = useVision;
  }
}
