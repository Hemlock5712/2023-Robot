// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class MoveElevatorExtension extends CommandBase {
  ExtensionSubsystem extension;
  double distance;

  /** Creates a new MoveElevatorExtension. */
  public MoveElevatorExtension(ExtensionSubsystem extension, double distance) {
    this.extension = extension;
    this.distance = distance;
    addRequirements(extension);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    extension.enableAutoDrive();
    extension.setTargetHeight(distance);
    // System.out.println("Setting distance: " + distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    extension.setTargetHeight(distance);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return extension.atTarget();
  }
}
