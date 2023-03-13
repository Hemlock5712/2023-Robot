// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.PiecePicker;

public class MidPlace extends CommandBase {
  ElevatorSubsystem elevator;
  ExtensionSubsystem extension;
  WristSubsystem wrist;
  Command command;
  /** Creates a new HighPlace. */
  public MidPlace(ElevatorSubsystem elevator, ExtensionSubsystem extension, WristSubsystem wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.extension = extension;
    this.wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(PiecePicker.getPiecePicker()){
      this.command = new MoveToSetpoint(elevator, extension, wrist,Constants.ArmSetpoints.MID_CUBE);
    }
    else{
      this.command = new MoveToSetpoint(elevator, extension, wrist,Constants.ArmSetpoints.MID_PEG);
    }
    this.command.schedule();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.command.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.command.isFinished();
  }
}
