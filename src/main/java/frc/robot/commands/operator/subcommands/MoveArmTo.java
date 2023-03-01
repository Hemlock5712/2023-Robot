package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FullArmSystem;
import frc.robot.util.ArmSetpoint;

public class MoveArmTo extends CommandBase {

  FullArmSystem arm;
  ArmSetpoint armSetpoint;

  public MoveArmTo(FullArmSystem armSystem, ArmSetpoint armSetpoint) {
    this.arm = armSystem;
    this.armSetpoint = armSetpoint;
  }

  public void execute() {
    arm.setTargetPosition(armSetpoint);
  }

  @Override
  public boolean isFinished() {
    return arm.atTarget();
  }
}
