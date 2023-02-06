// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class ReverseIntakeCommand extends CommandBase {
  TestSubsystem testSubsystem;

  /** Creates a new ReverseIntakeCommand. */
  public ReverseIntakeCommand(TestSubsystem testSubsystem) {
    this.testSubsystem = testSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(testSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    testSubsystem.reverseIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    testSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}