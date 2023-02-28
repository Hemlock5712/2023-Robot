package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FullArmSystem;

public class MoveArmToBottom extends CommandBase {

    FullArmSystem arm;

    public MoveArmToBottom(FullArmSystem armSystem) {
        this.arm = armSystem;
    }

    public void execute() {
        arm.setTargetPosition(Constants.ArmSetpoints.HYBRID_NODE);
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }
}
