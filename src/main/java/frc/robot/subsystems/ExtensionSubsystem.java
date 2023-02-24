package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.subsystems.ElevatorSubsystemBase;
import frc.robot.util.subsystems.ElevatorSubsystemBasePID;

public class ExtensionSubsystem extends ElevatorSubsystemBasePID {

    private TalonFX motor = new TalonFX(Constants.ExtensionConstants.EXTENSION_FALCON_ID);

    private NetworkTableEntry extensionLengthEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem").getEntry("extensionLength");
    private NetworkTableEntry motorVoltageEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem").getEntry("currentVoltage");

    public ExtensionSubsystem() {
        super(8, 0, 0, .05, .71, .2, 0.1, Constants.ExtensionConstants.EXTENSION_GEARING);
        motor.setInverted(true);
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
        this.setpoint = height;
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
        if (hasValidSetpoint) {
            return Math.abs(getHeight() - setpoint) < Constants.ExtensionConstants.AT_TARGET_TOLERANCE;
        }
        return true;
    }

    public void rawDrive(double percentage) {
        autoPosition = false;
        motor.set(ControlMode.PercentOutput, percentage);
    }

    public void enableAutoDrive() {
        autoPosition = true;
    }

    /**
     * Update NetworkTable entries for debugging purposes
     */
    @Override
    public void periodic() {
        super.periodic();
        extensionLengthEntry.setDouble(getHeight());
        motorVoltageEntry.setDouble(motor.getMotorOutputVoltage());
        SmartDashboard.putData(this);
    }
}
