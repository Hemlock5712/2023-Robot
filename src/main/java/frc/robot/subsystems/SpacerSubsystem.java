// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpacerSubsystem extends SubsystemBase {
  CANCoder angleEncoder = new CANCoder(Constants.SpacerConstants.CANCODER_ID);
  TalonFX SpacerMotor = new TalonFX(Constants.SpacerConstants.MOTOR_ID);
  PIDController pidController;
  double setpoint;

  /** Creates a new SpacerSubsystem. */
  public SpacerSubsystem() {
    SpacerMotor.setNeutralMode(NeutralMode.Coast);
    angleEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    angleEncoder.configMagnetOffset(Constants.ArmConstants.ARM_ANGLE_ABSOLUTE_OFFSET);
    angleEncoder.configSensorDirection(true);
    pidController = new PIDController(0.001, 0, 0);

  }

  public boolean atTarget() {
    return Math.abs(angleEncoder.getAbsolutePosition() - setpoint) < 0.5;
  }

  public void setMotorVoltage(double voltage) {
    SpacerMotor.set(ControlMode.PercentOutput, Math.min(.6, Math.max(-.3, voltage / 12)));
  }

  public void setAngle(double setpoint) {
    this.setpoint = setpoint;
  }

  @Override
  public void periodic() {
    pidController.setSetpoint(setpoint);
    double nextVoltage = pidController.calculate(angleEncoder.getAbsolutePosition());
    setMotorVoltage(nextVoltage);
    // This method will be called once per scheduler run
  }
}
