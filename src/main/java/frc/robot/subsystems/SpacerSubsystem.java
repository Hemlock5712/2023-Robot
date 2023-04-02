// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpacerSubsystem extends SubsystemBase {
  private CANSparkMax SpacerMotor = new CANSparkMax(Constants.SpacerConstants.MOTOR_ID,
      CANSparkMax.MotorType.kBrushless);
  PIDController pidController;

  /** Creates a new SpacerSubsystem. */
  public SpacerSubsystem() {
    SpacerMotor.setIdleMode(IdleMode.kCoast);
    SpacerMotor.setSmartCurrentLimit(20, 20);
    SpacerMotor.burnFlash();
    SpacerMotor.getEncoder().setPosition(0);
    pidController = new PIDController(0.4, 0, 0);

  }


  public void setMotorVoltage(double voltage) {
    SpacerMotor.setVoltage(voltage);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean pidPower(double angle) {
    pidController.setSetpoint(angle);
    double nextVoltage = pidController.calculate(SpacerMotor.getEncoder().getPosition(), angle);
    System.out.println(SpacerMotor.getEncoder().getPosition());
    System.out.println(nextVoltage);
    setMotorVoltage(nextVoltage);
    return Math.abs(SpacerMotor.getEncoder().getPosition() - angle) < 4;
  }

}
