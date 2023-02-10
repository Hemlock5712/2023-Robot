// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TestSubsystem extends SubsystemBase {

  // private CANSparkMax lower = new CANSparkMax(17, MotorType.kBrushed);
  // private CANSparkMax upper = new CANSparkMax(16, MotorType.kBrushed);

  private TalonFX intake = new TalonFX(33);
  private Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 0);

  public TestSubsystem() {
  }

  public void runIntake() {
    intake.set(ControlMode.PercentOutput, .8);
  }

  public void reverseIntake() {
    intake.set(ControlMode.PercentOutput, -.8);
  }

  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void openIntake() {
    claw.set(true);
  }

  public void closeIntake() {
    claw.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}