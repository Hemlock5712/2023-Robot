package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.subsystems.ElevatorSubsystemBasePID;

public class ExtensionSubsystem extends ElevatorSubsystemBasePID {

  private TalonFX motor = new TalonFX(Constants.ExtensionConstants.EXTENSION_FALCON_ID);

  AnalogInput stringPotentiometer = new AnalogInput(0);

  // Distance read by string potentiometer where it is considered as an error
  double stringPotentiometerErrorVoltage = .01;

  double stringPotentiometerBackupDistanceOffset = 0;

  private NetworkTableEntry extensionLengthEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem")
      .getEntry("extensionLength");
  private NetworkTableEntry targetLengthEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem")
      .getEntry("targetLength");
  private NetworkTableEntry motorVoltageEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem")
      .getEntry("currentVoltage");
  private NetworkTableEntry atSetpointEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem")
      .getEntry("atSetpoint");
  private NetworkTableEntry motorTempEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem")
      .getEntry("TempOfMotor");
  private NetworkTableEntry backupLengthEntry = NetworkTableInstance.getDefault().getTable("ExtensionSubsystem")
      .getEntry("backupLength");

  private boolean isRunningBackup = false;
  private boolean wasRunningBackup = false;

  public ExtensionSubsystem() {
    super(25, 0, 0, .05, .61, .2, 0.1, Constants.ExtensionConstants.EXTENSION_GEARING);
    motor.setInverted(true);

    stringPotentiometerBackupDistanceOffset = getHeight();
    if (stringPotentiometer.getVoltage() < stringPotentiometerErrorVoltage) {
      // Set the backup distance calculation to use the starting config if the string potentiometer is not working on startup
      // Also will send an alert to the driver station about there being an issue
      System.err.println("Error reading value from extension potentiometer. Defaulting to backup encoder with current length of " + Constants.ArmSetpoints.STARTING_CONFIG.getLength());
      stringPotentiometerBackupDistanceOffset = Constants.ArmSetpoints.STARTING_CONFIG.getLength();
    }
  }

  /**
   * Calculate the position pf the extension based on the string potentiometer
   *
   * This will be used in normal operation, but if the string potentiometer is not
   * working, it will use the backup encoder to calculate the position instead
   * 
   * @return extension length in meters
   */
  @Override
  public double getHeight() {
    double distance = ((stringPotentiometer.getVoltage()) * Constants.ExtensionConstants.MAX_EXTENSION
        / Constants.ExtensionConstants.MAX_VOLTAGE) - Constants.ExtensionConstants.MIN_EXTENSION;

    if(isRunningBackup || stringPotentiometer.getVoltage() < stringPotentiometerErrorVoltage) {
      isRunningBackup = true;
      return getBackupHeight();
    }

    return distance * 2;
  }

  /**
   * Calculate the position pf the extension based on the motor encoder
   *
   * This should only be used if the string potentiometer is not working
   *
   * @return extension length in meters
   */
  private double getBackupHeight() {
    double distance = motor.getSelectedSensorPosition() * Constants.ExtensionConstants.SPOOL_CIRCUMFERENCE
        / (Constants.ExtensionConstants.EXTENSION_GEARING * 4096) + stringPotentiometerBackupDistanceOffset;
    return distance * 2;
  }

  /**
   * Sets the target length that the extension should attempt to extend to
   *
   * The State Space model should automatically handle the movement to this
   * position
   */
  @Override
  public void setTargetHeight(double height) {
    hasValidSetpoint = true;
    this.setpoint = height;
  }

  /**
   * Set motor voltage
   * 
   * @param voltage voltage to set
   */
  @Override
  public void setMotorVoltage(double voltage) {
    motor.set(ControlMode.PercentOutput, voltage / 12.0);
  }

  @Override
  public boolean atTarget() {
    return Math.abs(getHeight() - setpoint) < Constants.ExtensionConstants.AT_TARGET_TOLERANCE;
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
    // System.out.println("ExtensionSubsystem.periodic() == " + setpoint);
    extensionLengthEntry.setDouble(getHeight());
    backupLengthEntry.setDouble(getBackupHeight());
    targetLengthEntry.setDouble(setpoint);
    motorVoltageEntry.setDouble(motor.getMotorOutputVoltage());
    motorTempEntry.setDouble(motor.getTemperature());
    SmartDashboard.putData(this);
    atSetpointEntry.setBoolean(atTarget());
    if(isRunningBackup != wasRunningBackup) {
      System.err.println("ElevatorSubsystem is running backup encoder. This needs to be looked at ASAP");
      wasRunningBackup = true;
    }
  }
}
