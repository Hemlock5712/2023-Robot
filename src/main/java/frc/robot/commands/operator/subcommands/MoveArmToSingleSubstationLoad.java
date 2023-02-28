package frc.robot.commands.operator.subcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FullArmSystem;

public class MoveArmToSingleSubstationLoad extends CommandBase {

    FullArmSystem arm;

    public MoveArmToSingleSubstationLoad(FullArmSystem armSystem) {
        this.arm = armSystem;
    }

    public void execute() {
        arm.setTargetPosition(Constants.ArmSetpoints.SINGLE_SUBSTATION_PICKUP);
    }

    @Override
    public boolean isFinished() {
        return arm.atTarget();
    }
}
