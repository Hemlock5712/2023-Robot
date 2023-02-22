package frc.robot.commands.operator.subcommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;

public class MoveArmToTop extends CommandBase {

    ElevatorSubsystem elevator;
    ExtensionSubsystem extension;

    public MoveArmToTop(ElevatorSubsystem elevator, ExtensionSubsystem extension) {
        this.elevator = elevator;
        this.extension = extension;
    }

    public void execute() {
        extension.setTargetHeight(Units.inchesToMeters(30));
        elevator.setTargetHeight(Units.inchesToMeters(40));
    }

    @Override
    public boolean isFinished() {
        return extension.atTarget() && elevator.atTarget();
    }
}
