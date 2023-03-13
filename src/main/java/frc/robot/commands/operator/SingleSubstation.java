// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class SingleSubstation extends CommandBase {
  ElevatorSubsystem elevator;
  ExtensionSubsystem extension;
  WristSubsystem wrist;
  IntakeSubsystem intake;
  Command command;

  /** Creates a new HighPlace. */
  public SingleSubstation(ElevatorSubsystem elevator, ExtensionSubsystem extension, WristSubsystem wrist,
      IntakeSubsystem intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.extension = extension;
    this.wrist = wrist;
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.command = new MoveToSetpoint(elevator,
        extension, wrist,
        Constants.ArmSetpoints.SINGLE_SUBSTATION_PICKUP);

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
