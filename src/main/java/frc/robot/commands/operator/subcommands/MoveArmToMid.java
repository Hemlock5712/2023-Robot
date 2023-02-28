package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FullArmSystem;

public class MoveArmToMid extends CommandBase {

    FullArmSystem arm;

    public MoveArmToMid(FullArmSystem armSystem) {
        this.arm = armSystem;
    }

    public void execute() {
        arm.setTargetPosition(Constants.ArmSetpoints.MID_PEG);
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }
}