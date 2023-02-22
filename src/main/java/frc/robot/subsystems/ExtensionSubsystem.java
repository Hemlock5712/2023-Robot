package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.util.subsystems.ElevatorSubsystemBase;

public class ExtensionSubsystem extends ElevatorSubsystemBase {

    private TalonFX motor = new TalonFX(Constants.ExtensionConstants.EXTENSION_FALCON_ID);

    private NetworkTableEntry extensionLengthEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem").getEntry("extensionLength");
    private NetworkTableEntry motorVoltageEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem").getEntry("currentVoltage");

    public ExtensionSubsystem() {
        super(
                DCMotor.getFalcon500(1),
                Constants.ExtensionConstants.SPOOL_RADIUS,
                Constants.ExtensionConstants.EXTENSION_MASS,
                Constants.ExtensionConstants.MAX_SPEED,
                Constants.ExtensionConstants.MAX_ACCELERATION,
                Constants.ExtensionConstants.EXTENSION_GEARING
        );
    }

    /**
     * Calculate the position pf the extension based on the motor encoder
     *
     * TODO: Use string potentiometer to get extension length
     * @return extension length in meters
     */
    @Override
    public double getHeight() {
        double distance = (motor.getSelectedSensorPosition() * Constants.ExtensionConstants.SPOOL_CIRCUMFERENCE) / (Constants.ExtensionConstants.EXTENSION_GEARING * 4096);
        return distance;
    }

    /**
     * Sets the target length that the extension should attempt to extend to
     *
     * The State Space model should automatically handle the movement to this position
     */
    @Override
    public void setTargetHeight(double height) {
        this.goal = new TrapezoidProfile.State(height, 0.0);
    }

    /**
     * Set motor voltage
     * @param voltage voltage to set
     */
    @Override
    public void setMotorVoltage(double voltage) {
        motor.set(ControlMode.PercentOutput, voltage / 12.0);
    }

    @Override
    public boolean atTarget() {
        return Math.abs(getHeight() - goal.position) < Constants.ExtensionConstants.AT_TARGET_TOLERANCE;
    }

    /**
     * Update NetworkTable entries for debugging purposes
     */
    @Override
    public void periodic() {
        super.periodic();
        extensionLengthEntry.setDouble(getHeight());
        motorVoltageEntry.setDouble(motor.getMotorOutputVoltage());
    }
}
