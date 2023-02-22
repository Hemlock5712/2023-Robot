package frc.robot.commands.operator.subcommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class MoveArmToMid extends CommandBase {

    ElevatorSubsystem elevator;
    ExtensionSubsystem extension;

    public MoveArmToMid(ElevatorSubsystem elevator, ExtensionSubsystem extension) {
        this.elevator = elevator;
        this.extension = extension;
    }

    public void execute() {
        extension.setTargetHeight(Units.inchesToMeters(20));
        elevator.setTargetHeight(Units.inchesToMeters(30));
    }

    @Override
    public boolean isFinished() {
        return extension.atTarget() && elevator.atTarget();
    }
}
