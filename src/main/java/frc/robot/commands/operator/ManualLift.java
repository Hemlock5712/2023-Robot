package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualLift extends CommandBase {
  ElevatorSubsystem elevator;
  double percentage;

  public ManualLift(ElevatorSubsystem elevatorSubsystem, double percentage) {
    this.elevator = elevatorSubsystem;
    this.percentage = percentage;
  }

  public void execute() {
    elevator.rawDrive(percentage);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.rawDrive(0);
  }
}
