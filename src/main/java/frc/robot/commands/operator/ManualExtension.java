package frc.robot.commands.operator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ManualExtension extends CommandBase {
  ExtensionSubsystem extension;
  double percentage;

  public ManualExtension(ExtensionSubsystem extensionSubsystem, double percentage) {
    this.extension = extensionSubsystem;
    this.percentage = percentage;
  }

  public void execute() {
    extension.rawDrive(percentage);
  }

  @Override
  public void end(boolean interrupted) {
    extension.rawDrive(0);
  }
}
