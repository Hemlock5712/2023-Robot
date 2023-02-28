package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.subsystems.ElevatorSubsystemBase;
import frc.robot.util.subsystems.ElevatorSubsystemBasePID;

public class ElevatorSubsystem extends ElevatorSubsystemBasePID {

  CANCoder angleEncoder = new CANCoder(Constants.ArmConstants.ARM_ANGLE_ENCODER_ID);
  TalonFX elevatorMotorFront = new TalonFX(Constants.ArmConstants.ELEVATOR_FRONT_FALCON_ID);
  TalonFX elevatorMotorBack = new TalonFX(Constants.ArmConstants.ELEVATOR_BACK_FALCON_ID);

  NetworkTableEntry angleEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("angle");
  NetworkTableEntry lengthEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("length");
  NetworkTableEntry motorVoltageEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem")
      .getEntry("currentVoltage");
  NetworkTableEntry motorCurrentEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem")
          .getEntry("motorCurrentDraw");
  NetworkTableEntry tempEntry = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("temperature");
  NetworkTableEntry targetHeight = NetworkTableInstance.getDefault().getTable("ElevatorSubsystem").getEntry("targetHeight");

  /**
   * Creates a new ElevatorSubsystem.
   *
   * This is a test using state space control to control the elevator,
   * but can easily be replaced with a PID controller
   */
  public ElevatorSubsystem() {
    super(8, 0, 0, .05, .71, .2, 0.1, Constants.ArmConstants.ELEVATOR_GEARING);
    elevatorMotorBack.follow(elevatorMotorFront);
    elevatorMotorFront.setInverted(true);
    elevatorMotorBack.setInverted(true);
    elevatorMotorBack.setNeutralMode(NeutralMode.Brake);
    elevatorMotorFront.setNeutralMode(NeutralMode.Brake);
  }

  /**
   * Gets the current height of the point where the elevator mounts with the arm
   * 
   * @return Current height, based on absolute CANCoder readings
   */
  @Override
  public double getHeight() {
    // Calculate the Y position of where the mounting point is, based off the
    // current angle
    double y = Math.sin(getAngle()) * Constants.ArmConstants.MOUNT_POINT_DISTANCE_ON_ARM;
    // Return the y component of the arm position, with the pivot point added.
    // This should give the absolute position of the arm mounting change
    return y + Constants.ArmConstants.PIVOT_POINT_HEIGHT;
  }

  /**
   * Sets the target height that the elevator should attempt to go to
   *
   * The PID control should automatically handle the movement to this
   * position
   */
  @Override
  public void setTargetHeight(double height) {
    hasValidSetpoint = true;
    this.setpoint = Math.min(Math.max(height, Constants.ArmConstants.MIN_ARM_LENGTH), Constants.ArmConstants.MAX_ARM_LENGTH);
  }

  /**
   * Sets the voltage of the elevator motor
   *
   * Technically this is a percent output, but it is scaled to 12V since Falcons
   * don't support voltage control
   *
   * Might be worth pulling the current voltage from the PDH and scaling it to
   * that for consistency
   * 
   * @param voltage Voltage to set the motor to
   */
  @Override
  public void setMotorVoltage(double voltage) {
    elevatorMotorFront.set(ControlMode.PercentOutput, Math.min(.6, Math.max(-.3, voltage / 12)));
  }

  @Override
  public boolean atTarget() {
    if(hasValidSetpoint) {
      return Math.abs(getHeight() - setpoint) < Constants.ArmConstants.AT_TARGET_TOLERANCE;
    }
    return true;
  }

  /**
   * Gets the current angle of the arm, with 0 being the horizontal position
   * parallel to the ground,
   * and positive angles being upwards
   *
   * @return Current angle, based on absolute CANCoder readings
   */
  public double getAngle() {
    return Units.degreesToRadians(angleEncoder.getAbsolutePosition() - Constants.ArmConstants.ARM_ANGLE_ABSOLUTE_OFFSET);
  }

  /**
   * Set the angle of the arm, with 0 being the horizontal position
   * @param angle
   */
  public void setAngle(double angle) {
    double y = Math.sin(angle) * Constants.ArmConstants.MOUNT_POINT_DISTANCE_ON_ARM;
    // Return the y component of the arm position, with the pivot point added.
    // This should give the absolute position of the arm mounting change
    double height =  y + Constants.ArmConstants.PIVOT_POINT_HEIGHT;

    setTargetHeight(height);
  }

  public void rawDrive(double percentage) {
    autoPosition = false;
    hasValidSetpoint = false;
    elevatorMotorFront.set(ControlMode.PercentOutput, percentage);
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
    angleEntry.setDouble(getAngle());
    lengthEntry.setDouble(getHeight());
    motorVoltageEntry.setDouble(elevatorMotorFront.getMotorOutputVoltage());
    motorCurrentEntry.setDouble(elevatorMotorFront.getStatorCurrent());
    SmartDashboard.putData(this);
    tempEntry.setDouble(elevatorMotorFront.getTemperature());
    targetHeight.setDouble(setpoint);
    SmartDashboard.putNumber("ElevatorSubsystem/PID", pidController.calculate(getHeight()));
    SmartDashboard.putNumber("ElevatorSubsystem/Feedforward", feedforward.calculate(getHeight()));
  }
}
