// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpacerSubsystem;

public class MoveSpacerAngle extends CommandBase {
  SpacerSubsystem spacerSubsystem;
  double angle;
  boolean atTarget = false;

  /** Creates a new MoveSpacerAngle. */
  public MoveSpacerAngle(double angle, SpacerSubsystem spacerSubsystem) {
    this.spacerSubsystem = spacerSubsystem;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(angle);
    this.atTarget = spacerSubsystem.pidPower(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.atTarget;
  }
}
