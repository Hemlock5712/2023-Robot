package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ManualExtensionIn extends CommandBase {
    ExtensionSubsystem extension;
    public ManualExtensionIn(ExtensionSubsystem extensionSubsystem) {
        this.extension = extensionSubsystem;
    }

    public void execute() {
        extension.rawDrive(-0.2);
    }

    @Override
    public void end(boolean interrupted) {
        extension.rawDrive(0);
    }
}
