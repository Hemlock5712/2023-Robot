package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class ManualLiftUp extends CommandBase {
    ElevatorSubsystem elevator;
    public ManualLiftUp(ElevatorSubsystem elevatorSubsystem) {
        this.elevator = elevatorSubsystem;
    }

    public void execute() {
        elevator.rawDrive(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        elevator.rawDrive(0);
    }
}
