// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MoveElevatorAngle extends CommandBase {
  ElevatorSubsystem elevator;
  double angle;

  /** Creates a new MoveElevatorAngle. */
  public MoveElevatorAngle(ElevatorSubsystem elevator, double angle) {
    this.elevator = elevator;
    this.angle = angle;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.enableAutoDrive();
    elevator.setAngle(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setAngle(angle);
    //System.out.println("Setting angle" + angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //System.out.println("Finished with MoveElevatorAngle");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atTarget();
  }
}
