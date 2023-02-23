package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualLiftDown extends CommandBase {
    ElevatorSubsystem elevator;
    public ManualLiftDown(ElevatorSubsystem elevatorSubsystem) {
        this.elevator = elevatorSubsystem;
    }

    @Override
    public void execute() {
        elevator.rawDrive(-0.1);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.rawDrive(0);
    }
}
