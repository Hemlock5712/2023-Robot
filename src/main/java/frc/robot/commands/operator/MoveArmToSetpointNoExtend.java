// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.operator.subcommands.MoveElevatorAngle;
import frc.robot.commands.operator.subcommands.MoveElevatorExtension;
import frc.robot.commands.operator.subcommands.MoveWristAngle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmSetpoint;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToSetpointNoExtend extends SequentialCommandGroup {
  /** Creates a new MoveArmToSetpointNoExtend. */
  public MoveArmToSetpointNoExtend(ElevatorSubsystem elevator, ExtensionSubsystem extension, WristSubsystem wrist,
      ArmSetpoint setpoint) {

    addCommands(
        new ParallelCommandGroup(
            new MoveElevatorAngle(elevator, Units.degreesToRadians(setpoint.getAngle())),
            new MoveWristAngle(wrist, setpoint.getWristAngle()),
            new MoveElevatorExtension(extension, 0)));
  }
}
