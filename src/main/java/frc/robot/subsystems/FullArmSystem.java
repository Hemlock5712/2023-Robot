package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ArmSetpoint;

public class FullArmSystem extends SubsystemBase {

    private ElevatorSubsystem elevator;
    private ExtensionSubsystem extension;
    private WristSubsystem wrist;

    /**
     * Provides control over the 3 main subsystems for moving the arm to where it should be.
     *
     * This is a convenience class that allows us to control all 3 subsystems at once.
     * @param elevator Elevator subsystem
     * @param extension Extension subsystem
     * @param wrist Wrist subsystem
     */
    public FullArmSystem(ElevatorSubsystem elevator, ExtensionSubsystem extension, WristSubsystem wrist) {
        this.elevator = elevator;
        this.extension = extension;
        this.wrist = wrist;
    }

    /**
     * Sets the target position for the arm.
     * @param setpoint The setpoint to move to.
     */
    public void setTargetPosition(ArmSetpoint setpoint) {
        elevator.setAngle(setpoint.getAngle());
        extension.setTargetHeight(setpoint.getLength());
        wrist.setTargetAngle(setpoint.getWristAngle());
    }

    /**
     * Checks if the arm is at the target position.
     *
     * Use this for autonomous commands to determine when the arm is at the target position
     * and the autonomous flow can continue.
     *
     * @return True if the arm is at the target position, false otherwise.
     */
    public boolean atTarget() {
        return elevator.atTarget() && extension.atTarget() && wrist.atTarget();
    }
}
