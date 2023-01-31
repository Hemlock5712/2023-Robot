// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  private CANSparkMax lower = new CANSparkMax(17, MotorType.kBrushed);
  private CANSparkMax upper = new CANSparkMax(16, MotorType.kBrushed);

  public TestSubsystem() {
  }

  public void runIntake() {
    lower.set(0.5);
    upper.set(-.5);
  }

  public void reverseIntake() {
    lower.set(-1);
    upper.set(1);
  }

  public void stopIntake() {
    lower.set(0);
    upper.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}