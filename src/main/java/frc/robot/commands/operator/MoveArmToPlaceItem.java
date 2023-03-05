// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.operator.subcommands.MoveElevatorAngle;
import frc.robot.commands.operator.subcommands.MoveElevatorExtension;
import frc.robot.commands.operator.subcommands.MoveWristAngle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.ArmSetpoint;
import frc.robot.util.TargetLevel;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveArmToPlaceItem extends CommandBase {

  ElevatorSubsystem elevator;
  ExtensionSubsystem extension;
  WristSubsystem wrist;
  boolean extend;

  /** Creates a new MoveArmToSetpoint. */
  public MoveArmToPlaceItem(ElevatorSubsystem elevator, ExtensionSubsystem extension, WristSubsystem wrist, boolean extend) {
    this.elevator = elevator;
    this.extension = extension;
    this.wrist = wrist;
    this.extend = extend;
    addRequirements(elevator, extension, wrist);
  }

  Command wristCommand = null;
  Command elevatorCommand = null;
  Command extensionCommand = null;

  @Override
  public void initialize() {
    TargetLevel level = Position.getPlacementPosition().getLevel();

    Command goToEndArmPosition = null;
    Command goToEndArmPositionTransit = null;
    ArmSetpoint setpoint = null;

    switch(level) {
      case Top:
        goToEndArmPosition = new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.HIGH_PEG);
        goToEndArmPositionTransit = new MoveArmToSetpointNoExtend(elevator, extension, wrist, Constants.ArmSetpoints.HIGH_PEG);
        setpoint = Constants.ArmSetpoints.HIGH_PEG;
        break;
      case Mid:
        goToEndArmPosition = new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.MID_PEG);
        goToEndArmPositionTransit = new MoveArmToSetpointNoExtend(elevator, extension, wrist, Constants.ArmSetpoints.MID_PEG);
        setpoint = Constants.ArmSetpoints.MID_PEG;
        break;
      case Low:
        goToEndArmPosition = new MoveArmToSetpoint(elevator, extension, wrist, Constants.ArmSetpoints.HYBRID_NODE);
        goToEndArmPositionTransit = new MoveArmToSetpointNoExtend(elevator, extension, wrist, Constants.ArmSetpoints.HYBRID_NODE);
        setpoint = Constants.ArmSetpoints.HYBRID_NODE;
        break;
    }

    elevatorCommand = new MoveElevatorAngle(elevator, Units.degreesToRadians(setpoint.getAngle()));
    wristCommand = new MoveWristAngle(wrist, setpoint.getWristAngle());
    extensionCommand = new MoveElevatorExtension(extension, extend ? setpoint.getLength() : 0);

    elevatorCommand.schedule();
    wristCommand.schedule();
    extensionCommand.schedule();
  }

    @Override
    public void end(boolean interrupted) {
      if (elevatorCommand != null) {
        elevatorCommand.cancel();
      }
      if (wristCommand != null) {
        wristCommand.cancel();
      }
      if (extensionCommand != null) {
        extensionCommand.cancel();
      }
    }

    @Override
    public boolean isFinished() {
      return extensionCommand.isFinished() && wristCommand.isFinished() && elevatorCommand.isFinished();
    }
}
