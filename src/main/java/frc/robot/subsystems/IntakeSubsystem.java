// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // private CANSparkMax lower = new CANSparkMax(17, MotorType.kBrushed);
  // private CANSparkMax upper = new CANSparkMax(16, MotorType.kBrushed);

  private TalonFX intake = new TalonFX(33);
  private Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 0);
  private double speedSetpoint = 0;

  private PIDController intakePID = new PIDController(1, 0, 0);
  private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0.01, 1.18, 0.01);

  public IntakeSubsystem() {
    intake.setInverted(true);
  }

  public void runIntake() {
    if (claw.get()) {
      intake.set(ControlMode.PercentOutput, -0.1);
      speedSetpoint = 15;
    } else {
      intake.set(ControlMode.PercentOutput, -1);
      speedSetpoint = 1500;
    }
  }

  public void reverseIntake() {
    speedSetpoint = -800;
    intake.set(ControlMode.PercentOutput, .8);
  }

  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
    speedSetpoint = 0;
  }

  public void openIntake() {
    claw.set(true);
  }

  public void closeIntake() {
    claw.set(false);
  }

  @Override
  public void periodic() {
    double currentSpeed = intake.getSelectedSensorVelocity();
    double targetVoltage = (intakePID.calculate(currentSpeed, speedSetpoint)
        + intakeFF.calculate(currentSpeed, speedSetpoint, 0.2)) / 12.0;
    intake.set(ControlMode.PercentOutput,
        targetVoltage);
    intake.set(ControlMode.PercentOutput, 0);
    SmartDashboard.putNumber("Intake/TargetVoltage", intake.getMotorOutputPercent());
    SmartDashboard.putNumber("Intake/TargetSpeed", speedSetpoint);
    speedSetpoint = 0;
    // This method will be called once per scheduler run
  }
}