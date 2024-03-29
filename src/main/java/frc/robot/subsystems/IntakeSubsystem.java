// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

  // private CANSparkMax lower = new CANSparkMax(17, MotorType.kBrushed);
  // private CANSparkMax upper = new CANSparkMax(16, MotorType.kBrushed);

  private LEDSubsystem ledSubsystem;

  private TalonFX intake = new TalonFX(33);
  private Solenoid claw = new Solenoid(PneumaticsModuleType.REVPH, 8);
  // private NetworkTableEntry motorTempEntry =
  // NetworkTableInstance.getDefault().getTable("Intake")
  // .getEntry("temperature");
  // private double speedSetpoint = 0;

  // private PIDController intakePID = new PIDController(1, 0, 0);
  // private SimpleMotorFeedforward intakeFF = new SimpleMotorFeedforward(0.01,
  // 1.18, 0.01);

  public IntakeSubsystem(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    intake.setInverted(false);
  }

  public void runIntake() {
    intake.set(ControlMode.PercentOutput, -1);
    // speedSetpoint = 1500;
  }

  public void holdCone() {
    intake.set(ControlMode.PercentOutput, -.2);
  }

  public void holdCube() {
    intake.set(ControlMode.PercentOutput, -.1);
  }

  public void reverseIntake() {
    // speedSetpoint = -800;
    intake.set(ControlMode.PercentOutput, .8);
  }

  public void reverseIntakeCone() {
    // speedSetpoint = -800;
    intake.set(ControlMode.PercentOutput, .4);
  }

  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
    // speedSetpoint = 0;
  }

  public void openIntake() {
    claw.set(true);
  }

  public void closeIntake() {
    claw.set(false);
  }

  @Override
  public void periodic() {
    // double currentSpeed = intake.getSelectedSensorVelocity();
    // double targetVoltage = (intakePID.calculate(currentSpeed, speedSetpoint)
    // + intakeFF.calculate(currentSpeed, speedSetpoint, 0.2)) / 12.0;
    // intake.set(ControlMode.PercentOutput,
    // targetVoltage);
    // intake.set(ControlMode.PercentOutput, 0);
    // SmartDashboard.putNumber("Intake/TargetVoltage",
    // intake.getMotorOutputPercent());
    // SmartDashboard.putNumber("Intake/TargetSpeed", speedSetpoint);
    // speedSetpoint = 0;
    // This method will be called once per scheduler run
    // motorTempEntry.setDouble(intake.getTemperature());
    // SmartDashboard.putNumber("Intake Current", intake.getSupplyCurrent());
    // SmartDashboard.putNumber("Intake Speed", intake.getSelectedSensorVelocity());
    if (Math.abs(intake.getMotorOutputPercent()) > .01) {
      if (Math.abs(intake.getSelectedSensorVelocity()) < 500) {
        ledSubsystem.setIsHoldingGamePiece(true);
      } else {
        ledSubsystem.setIsHoldingGamePiece(false);
      }
    } else {
      ledSubsystem.setIsHoldingGamePiece(false);
    }
  }
}