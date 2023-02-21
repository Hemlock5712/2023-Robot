package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestSubsystem;

public class OpenClaw extends CommandBase {
    TestSubsystem testSubsystem;

    /** Creates a new ReverseIntakeCommand. */
    public OpenClaw(TestSubsystem testSubsystem) {
        this.testSubsystem = testSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        testSubsystem.openIntake();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        testSubsystem.closeIntake();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
