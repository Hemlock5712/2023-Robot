// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.balance.AutoBalance;
import frc.robot.commands.operator.MoveToSetpoint;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PoseEstimatorSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousCenterBalance extends SequentialCommandGroup {
  /** Creates a new AutonomousCenterBalance. */
  public AutonomousCenterBalance(DrivetrainSubsystem drivetrain, PoseEstimatorSubsystem poseEstimator,
      ElevatorSubsystem elevator, ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
        new MoveToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.HIGH_CUBE),
        new ReverseIntakeCommand(intake).withTimeout(0.5),
        new MoveToSetpoint(elevator, extension, wrist, new ArmSetpoint(30, 0, 45))
            .withTimeout(0.5).andThen(
                new MoveToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.TRANSIT)
                    .withTimeout(0.5)),
        new AutoBalance(drivetrain, poseEstimator));
  }
}
