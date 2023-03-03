// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class MoveWristAngle extends CommandBase {
  WristSubsystem wrist;
  double angle;

  /** Creates a new MoveElevatorAngle. */
  public MoveWristAngle(WristSubsystem wrist, double angle) {
    this.wrist = wrist;
    this.angle = angle;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setTargetAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setTargetAngle(angle);
    System.out.println("Setting angle" + angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Finished with MoveElevatorAngle");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return wrist.atTarget();
  }
}
