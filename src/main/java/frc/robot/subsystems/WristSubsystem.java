package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WristSubsystem extends SubsystemBase {
  private CANSparkMax wristMotor = new CANSparkMax(Constants.WristConstants.MOTOR_ID, CANSparkMax.MotorType.kBrushless);
  private CANCoder wristEncoder = new CANCoder(Constants.WristConstants.ENCODER_ID);
  // These constants are lower than they should be to prevent the wrist from going
  // too far instantly
  private PIDController wristPID = new PIDController(.1, 0, 0);
  // These constants are calculated by Reca.lc, might need to be tuned slightly
  // private ArmFeedforward wristFeedforward = new ArmFeedforward(0, 2.62, 0.48,
  // 0.07);
  private ArmFeedforward wristFeedforward = new ArmFeedforward(0, .2, .79, .04);

  private NetworkTableEntry wristTargetAngleEntry = NetworkTableInstance.getDefault().getTable("Wrist")
      .getEntry("targetAngle");
  private NetworkTableEntry wristCurrentAngleEntry = NetworkTableInstance.getDefault().getTable("Wrist")
      .getEntry("currentAngle");
  private NetworkTableEntry wristVoltageEntry = NetworkTableInstance.getDefault().getTable("Wrist").getEntry("voltage");
  private NetworkTableEntry wristCurrentEntry = NetworkTableInstance.getDefault().getTable("Wrist").getEntry("current");
  private NetworkTableEntry wristTemperatureEntry = NetworkTableInstance.getDefault().getTable("Wrist")
      .getEntry("termperature");
  private NetworkTableEntry atSetpointEntry = NetworkTableInstance.getDefault().getTable("Wrist")
      .getEntry("atSetpoint");

  private double setpoint = 0;

  public WristSubsystem() {
    wristEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    wristEncoder.configMagnetOffset(-50);
    wristMotor.setIdleMode(IdleMode.kBrake);
    wristMotor.setSmartCurrentLimit(80);
    // wristMotor.setSmartCurrentLimit(30, 40);

  }

  public void setTargetAngle(double angle) {
    this.setpoint = angle;
  }

  public double getTargetAngle() {
    return this.setpoint;
  }

  public double getAngle() {
    // We need to find a set point to start the wrist at as a 0 point
    // return wristMotor.getEncoder().getPosition() * 360
    // / (Constants.WristConstants.TICKS_PER_REVOLUTION *
    // Constants.WristConstants.GEAR_RATIO);
    return wristEncoder.getAbsolutePosition();
  }

  public void setVoltage(double voltage) {
    wristMotor.setVoltage(voltage);
  }

  public boolean atTarget() {
    return Math.abs(getAngle() - setpoint) < 4;
  }

  public void run() {
    double feedforward = wristFeedforward.calculate(Math.toRadians(setpoint), 0);
    // + feedforward
    // double output = feedforward;
    double pid = wristPID.calculate(getAngle(), setpoint);
    SmartDashboard.putNumber("WristPID", pid);
    SmartDashboard.putNumber("WristFF", feedforward);
    double output = feedforward + pid;
    SmartDashboard.putNumber("WristOutput", output);
    // System.out.println(feedforward);
    // System.out.println("_________ " + output);

    setVoltage(output);
    wristCurrentAngleEntry.setDouble(getAngle());
    wristTargetAngleEntry.setDouble(setpoint);
    wristVoltageEntry.setDouble(wristMotor.getAppliedOutput());
    wristCurrentEntry.setDouble(wristMotor.getOutputCurrent());
    atSetpointEntry.setBoolean(atTarget());
  }

  @Override
  public void periodic() {
    run();
    // double feedforward = wristFeedforward.calculate(setpoint, 0);
    // double output = wristPID.calculate(getAngle(), setpoint) + feedforward;
    // // setVoltage(output);
    // wristCurrentAngleEntry.setDouble(getAngle());
    // wristTargetAngleEntry.setDouble(setpoint);
    // wristVoltageEntry.setDouble(wristMotor.getAppliedOutput());
    // wristCurrentEntry.setDouble(wristMotor.getOutputCurrent());
    wristTemperatureEntry.setDouble(wristMotor.getMotorTemperature());

  }
}
