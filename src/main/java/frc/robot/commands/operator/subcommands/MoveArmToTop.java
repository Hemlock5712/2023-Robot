package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FullArmSystem;

public class MoveArmToTop extends CommandBase {

    FullArmSystem arm;

    public MoveArmToTop(FullArmSystem armSystem) {
        this.arm = armSystem;
    }

    public void execute() {
        arm.setTargetPosition(Constants.ArmSetpoints.HIGH_PEG);
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }
}
